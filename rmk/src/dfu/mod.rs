#[cfg(feature = "_dfu")]
use core::sync::atomic::AtomicBool;
#[cfg(feature = "_dfu")]
use core::sync::atomic::AtomicU32;
#[cfg(feature = "_dfu")]
use core::sync::atomic::Ordering;

// ---------------------------------------------------------------------------
// Flash partition types and constants
// ---------------------------------------------------------------------------
#[cfg(feature = "_dfu")]
use embassy_boot::{FirmwareState, FirmwareUpdater, FirmwareUpdaterConfig};
#[cfg(feature = "_dfu")]
pub use embassy_embedded_hal::flash::partition::Partition;
#[cfg(feature = "_dfu")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "_dfu")]
use embassy_sync::channel::Channel;
#[cfg(feature = "dfu_lock")]
use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::NorFlash;
#[cfg(feature = "_dfu")]
use heapless;
#[cfg(feature = "dfu_lock")]
use rmk_types::dfu::DfuStatus;
use static_cell::StaticCell;

#[cfg(feature = "_dfu")]
use crate::core_traits::Runnable;
#[cfg(feature = "dfu_lock")]
use crate::event::publish_event;

/// Total flash size passed to the embassy-rp Flash const generic.
///
/// Set to 16 MB (the maximum common RP2040 flash size) so that the same
/// binary works on boards with 2, 4, 8 or 16 MB flash.  `new_blocking()`
/// ignores this value at runtime — it is only used for software bounds
/// checking inside embassy-rp.  Because all flash access goes through
/// `Partition` (which has its own partition-sized bounds checks),
/// overshooting the const generic is safe.
#[cfg(feature = "dfu_rp")]
pub const FLASH_SIZE: usize = 16 * 1024 * 1024;

/// Capacity of the aligned scratch buffer used by embassy-boot. Must be >=
/// the largest `WRITE_SIZE` among all supported flash types (RP2040 internal
/// flash and 25-series SPI NOR: 1; nRF NVMC: 4). The buffer is sliced to the
/// state partition's exact `WRITE_SIZE` before being handed to embassy-boot.
#[cfg(feature = "_dfu")]
pub const DFU_WRITE_SIZE: usize = 256;

/// Block size of a DFU download transferred per USB control request.
/// Larger values speed up firmware downloads. Must match the USB control
/// buffer size used by the host.
pub const BLOCK_SIZE_DFU: usize = 512;

/// Partition layout read from the DFU symbols in `memory.x`.
///
/// The offsets are flash-relative and come from the `__bootloader_*` symbols
/// that rmk-boot's generated `memory.x` provides.
#[cfg(feature = "_dfu")]
#[derive(Clone, Copy, Debug)]
pub struct DfuFlashLayout {
    /// Offset of the DFU download partition.
    pub dfu_offset: u32,
    /// Size of the DFU download partition.
    pub dfu_size: u32,
    /// Offset of the boot state partition (holds the embassy-boot state flags).
    pub state_offset: u32,
    /// Size of the boot state partition.
    pub state_size: u32,
    /// Offset of the storage partition.
    pub storage_offset: u32,
    /// Size of the storage partition.
    pub storage_size: u32,
}

/// Read the partition layout from the DFU symbols in `memory.x`.
///
/// # Safety
///
/// Reads linker-defined absolute symbols. The symbols must be present in the
/// linked binary or the firmware will not link.
#[cfg(feature = "_dfu")]
pub fn dfu_flash_layout() -> DfuFlashLayout {
    unsafe extern "C" {
        static __bootloader_state_start: u8;
        static __bootloader_state_end: u8;
        static __bootloader_dfu_start: u8;
        static __bootloader_dfu_end: u8;
        static __bootloader_storage_start: u8;
        static __bootloader_storage_end: u8;
    }
    // SAFETY: linker-defined symbols — reading their addresses is safe.
    DfuFlashLayout {
        dfu_offset: core::ptr::addr_of!(__bootloader_dfu_start) as usize as u32,
        dfu_size: core::ptr::addr_of!(__bootloader_dfu_end) as usize as u32
            - core::ptr::addr_of!(__bootloader_dfu_start) as usize as u32,
        state_offset: core::ptr::addr_of!(__bootloader_state_start) as usize as u32,
        state_size: core::ptr::addr_of!(__bootloader_state_end) as usize as u32
            - core::ptr::addr_of!(__bootloader_state_start) as usize as u32,
        storage_offset: core::ptr::addr_of!(__bootloader_storage_start) as usize as u32,
        storage_size: core::ptr::addr_of!(__bootloader_storage_end) as usize as u32
            - core::ptr::addr_of!(__bootloader_storage_start) as usize as u32,
    }
}

/// Mutex guarding the flash, shared by all partitions.
#[cfg(feature = "_dfu")]
pub type FlashMutex<F> = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, F>;

/// A partition of the flash (DFU download, boot state, or storage).
#[cfg(feature = "_dfu")]
pub type DfuPartition<'a, F> = Partition<'a, CriticalSectionRawMutex, F>;

/// The storage partition — same as DfuPartition now (async, no wrapper needed).
#[cfg(feature = "_dfu")]
pub type DfuStorage<'a, F> = DfuPartition<'a, F>;

/// Build the storage, boot state and DFU download partitions from the
/// `memory.x` layout (see [`dfu_flash_layout`]).
///
/// Returns `(storage, state, dfu)` partitions over the same flash mutex.
/// `storage` is async — pass it straight to the keymap/storage layer.
/// `state` feeds [`mark_booted`]; `dfu` and `state` go into
/// [`RmkDfuInterface::new`].
///
/// When the DFU download partition lives on an external flash (`dfu_ext`),
/// discard the returned `dfu` partition and build the external one yourself.
#[cfg(feature = "_dfu")]
pub fn partitions_from_linkerscript<'a, F: NorFlash>(
    flash_mutex: &'a FlashMutex<F>,
) -> (DfuStorage<'a, F>, DfuPartition<'a, F>, DfuPartition<'a, F>) {
    let layout = dfu_flash_layout();
    let storage = DfuPartition::new(flash_mutex, layout.storage_offset, layout.storage_size);
    let state = DfuPartition::new(flash_mutex, layout.state_offset, layout.state_size);
    let dfu = DfuPartition::new(flash_mutex, layout.dfu_offset, layout.dfu_size);
    (storage, state, dfu)
}

/// Mark firmware boot as successful so the bootloader doesn't revert the
/// update on the next reset.
///
/// `state` is the boot state partition — typically built with
/// [`partitions_from_linkerscript`].
///
/// Must be called *after* the firmware is confirmed running, before the
/// bootloader's timeout would consider the update failed.
#[cfg(feature = "_dfu")]
pub async fn mark_booted<STATE: NorFlash>(state: &mut STATE) {
    static ALIGNED: StaticCell<[u8; DFU_WRITE_SIZE]> = StaticCell::new();
    let mut firmware_state = FirmwareState::new(state, &mut ALIGNED.init([0; DFU_WRITE_SIZE])[..STATE::WRITE_SIZE]);
    firmware_state.mark_booted().await.ok();
}

// ---------------------------------------------------------------------------
// dfu_split sub-module (behind feature flag)
// ---------------------------------------------------------------------------

#[cfg(feature = "dfu_split")]
mod split;
#[cfg(feature = "dfu_split")]
pub(crate) use self::split::PassthroughDfuHandler;
#[cfg(feature = "dfu_split")]
pub(crate) use self::split::{
    PASSTHROUGH_CHANNEL, PASSTHROUGH_TARGET, PassthroughCommand, passthrough_done_if_empty, passthrough_peek,
    passthrough_pending, passthrough_take_command,
};
#[cfg(feature = "dfu_split")]
pub use self::split::{
    SplitDfuHandler, get_firmware_update_data, read_embedded_firmware_hash, set_firmware_update_data,
};

// ---------------------------------------------------------------------------
// DFU command channel (USB proxy → updater task)
// ---------------------------------------------------------------------------

/// Command queue capacity — one USB control block plus slack.
#[cfg(feature = "_dfu")]
const DFU_CMD_QUEUE_SIZE: usize = 2;

/// A command forwarded from the USB DFU proxy to the async updater task.
#[cfg(feature = "_dfu")]
pub(crate) enum DfuCmd {
    Start,
    Write(heapless::Vec<u8, { BLOCK_SIZE_DFU }>),
    Finish,
    SystemReset,
}

/// Command channel: the USB DFU proxy (ISR context) sends via
/// [`DFU_CHANNEL`]; the [`RmkDfuInterface`] updater task receives.
#[cfg(feature = "_dfu")]
pub(crate) static DFU_CHANNEL: Channel<CriticalSectionRawMutex, DfuCmd, DFU_CMD_QUEUE_SIZE> = Channel::new();

/// Doorbell atomic: true while the command queue is non-empty. Read by the
/// transport's GETSTATUS handling to inject `dfuDNBUSY` (adaptive host-side
/// flow control, mirroring [`PASSTHROUGH_TARGET`]).
#[cfg(feature = "_dfu")]
pub(crate) static DFU_BUSY: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "_dfu")]
pub(crate) static DFU_WRITE_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Push a command into the queue (ISR-safe). Returns `Err(())` when full.
#[cfg(feature = "_dfu")]
pub(crate) fn dfu_push(cmd: DfuCmd) -> Result<(), ()> {
    DFU_CHANNEL.try_send(cmd).map_err(|_| ())?;
    DFU_BUSY.store(true, Ordering::Release);
    Ok(())
}

/// Clear the busy flag if the queue has drained.
///
/// Called by the updater task after every drain cycle. The host stays in
/// `dfuDNBUSY` until the queue catches up.
#[cfg(feature = "_dfu")]
pub(crate) fn dfu_clear_busy_if_empty() {
    if DFU_CHANNEL.is_empty() {
        DFU_BUSY.store(false, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// DFU lock state
// ---------------------------------------------------------------------------

#[cfg(feature = "dfu_lock")]
static DFU_LOCKED: AtomicBool = AtomicBool::new(true);
#[cfg(feature = "dfu_lock")]
static DFU_STARTED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "dfu_lock")]
static DFU_UNLOCK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[cfg(feature = "dfu_lock")]
pub fn is_dfu_unlocked() -> bool {
    !DFU_LOCKED.load(Ordering::Acquire)
}

/// Gate shared by the transport's DFU start handlers (central alt 0 and the
/// passthrough slots). Returns `Ok` when a download may proceed and records
/// that it started; while the keys are locked it wakes the unlock state
/// machine and rejects the download with `ErrVendor`.
#[cfg(feature = "_dfu")]
pub(crate) fn dfu_lock_check() -> Result<(), embassy_usb::class::dfu::consts::Status> {
    #[cfg(feature = "dfu_lock")]
    {
        use embassy_usb::class::dfu::consts::Status;
        if !is_dfu_unlocked() {
            DFU_UNLOCK_SIGNAL.signal(());
            info!("dfu_lock: DFU download rejected — keys not unlocked");
            return Err(Status::ErrVendor);
        }
        DFU_STARTED.store(true, Ordering::Release);
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// RmkDfuInterface — partition-based DFU updater task
// ---------------------------------------------------------------------------

/// Max passthrough alt settings supported on a single DFU interface.
#[cfg(feature = "dfu_split")]
pub(crate) const MAX_PASSTHROUGH_ALTS: usize = 4;

/// Flash-side DFU updater.
///
/// Owns the embassy-boot async [`FirmwareUpdater`] and runs as a [`Runnable`]
/// task. It waits on the command channel ([`DFU_CHANNEL`]) and executes
/// `start`/`write`/`finish`/`system_reset` on the updater, fully decoupled
/// from the USB device.
///
/// The USB side (the proxy in `usb.rs`) never touches flash; all commands flow
/// through the channel. The partitions are typically built with
/// [`partitions_from_linkerscript`]:
///
/// ```
/// let flash_mutex = ::rmk::dfu::FlashMutex::new(flash_driver);
/// let (_, mut state_partition, dfu_partition) =
///     ::rmk::dfu::partitions_from_linkerscript(&flash_mutex);
/// let mut dfu_iface = ::rmk::dfu::RmkDfuInterface::new(dfu_partition, state_partition);
/// ```
#[cfg(feature = "_dfu")]
pub struct RmkDfuInterface<DFU: NorFlash, STATE: NorFlash> {
    updater: FirmwareUpdater<'static, DFU, STATE>,
    offset: u32,
    write_errors: u32,
}

#[cfg(feature = "_dfu")]
impl<DFU: NorFlash, STATE: NorFlash> RmkDfuInterface<DFU, STATE> {
    /// Build the DFU updater from a DFU download partition and a boot state
    /// partition.
    pub fn new(dfu: DFU, state: STATE) -> Self {
        const { core::assert!(STATE::WRITE_SIZE <= DFU_WRITE_SIZE) };
        let config = FirmwareUpdaterConfig { dfu, state };
        static ALIGNED: StaticCell<[u8; DFU_WRITE_SIZE]> = StaticCell::new();
        let aligned: &'static mut [u8] = &mut ALIGNED.init([0; DFU_WRITE_SIZE])[..STATE::WRITE_SIZE];
        let updater = FirmwareUpdater::new(config, aligned);
        Self {
            updater,
            offset: 0,
            write_errors: 0,
        }
    }
}

#[cfg(feature = "_dfu")]
impl<DFU: NorFlash, STATE: NorFlash> Runnable for RmkDfuInterface<DFU, STATE> {
    async fn run(&mut self) -> ! {
        loop {
            let cmd = DFU_CHANNEL.receive().await;
            self.handle_cmd(cmd).await;
            while let Ok(cmd) = DFU_CHANNEL.try_receive() {
                self.handle_cmd(cmd).await;
            }
            dfu_clear_busy_if_empty();
        }
    }
}

#[cfg(feature = "_dfu")]
impl<DFU: NorFlash, STATE: NorFlash> RmkDfuInterface<DFU, STATE> {
    async fn handle_cmd(&mut self, cmd: DfuCmd) {
        match cmd {
            DfuCmd::Start => {
                self.offset = 0;
                self.write_errors = 0;
                DFU_WRITE_ERRORS.store(0, Ordering::Release);
                match self.updater.get_state().await {
                    Ok(_) => info!("dfu: state ok"),
                    Err(_) => error!("dfu: get_state failed"),
                }
            }
            DfuCmd::Write(data) => match self.updater.write_firmware(self.offset as usize, &data).await {
                Ok(()) => self.offset += data.len() as u32,
                Err(_) => {
                    error!("dfu: firmware write failed");
                    self.write_errors += 1;
                    self.offset += data.len() as u32;
                    DFU_WRITE_ERRORS.store(self.write_errors, Ordering::Release);
                }
            },
            DfuCmd::Finish => {
                if self.write_errors > 0 {
                    error!("dfu: update aborted - {} write errors occurred", self.write_errors);
                    self.write_errors = 0;
                } else {
                    info!("dfu: {} bytes written, verifying...", self.offset);
                    let mut hdr = [0u8; 8];
                    if self.updater.read_dfu(0, &mut hdr).await.is_ok() {
                        info!("dfu: DFU[0..8] = {:02x}", hdr);
                    }
                    let all_ff = hdr.iter().all(|&b| b == 0xFF);
                    let all_00 = hdr.iter().all(|&b| b == 0x00);
                    let msp = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
                    let reset = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
                    if all_ff || all_00 || msp == 0 || msp == 0xFFFF_FFFF || reset == 0 || reset == 0xFFFF_FFFF
                    {
                        error!(
                            "dfu: sanity check failed (msp={:#010x}, reset={:#010x}), skipping mark_updated",
                            msp, reset
                        );
                        self.write_errors += 1;
                        DFU_WRITE_ERRORS.store(self.write_errors, Ordering::Release);
                    } else {
                        match self.updater.mark_updated().await {
                            Ok(()) => info!("dfu: update complete, resetting"),
                            Err(_) => error!("dfu: firmware finish failed"),
                        }
                    }
                }
            }
            DfuCmd::SystemReset => {
                #[cfg(all(
                    target_arch = "arm",
                    target_os = "none",
                    any(target_abi = "eabi", target_abi = "eabihf")
                ))]
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// dfu_lock
// ---------------------------------------------------------------------------

/// DfuLock state machine that checks a physical key combination to unlock DFU.
#[cfg(feature = "dfu_lock")]
pub struct DfuLock<'a> {
    unlocked: AtomicBool,
    unlock_keys: &'a [(u8, u8)],
    keymap: &'a crate::keymap::KeyMap<'a>,
}

#[cfg(feature = "dfu_lock")]
impl<'a> DfuLock<'a> {
    pub fn new(unlock_keys: &'a [(u8, u8)], keymap: &'a crate::keymap::KeyMap<'a>) -> Self {
        Self {
            unlocked: AtomicBool::new(false),
            unlock_keys,
            keymap,
        }
    }

    pub(crate) async fn process_unlock(&self) {
        DFU_UNLOCK_SIGNAL.wait().await;

        info!("dfu_lock: DFU activity detected, unlock window open for 10 s");
        info!("dfu_lock: waiting for unlock keys");
        publish_event(crate::event::DfuStatusEvent::new(DfuStatus::LockWaiting));
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(10);
        loop {
            let all_pressed = self
                .unlock_keys
                .iter()
                .all(|(row, col)| self.keymap.read_matrix_key(*row, *col));
            if all_pressed {
                self.unlocked.store(true, Ordering::Release);
                DFU_LOCKED.store(false, Ordering::Release);
                info!("dfu_lock: unlock keys pressed, DFU unlocked for 10 s");
                publish_event(crate::event::DfuStatusEvent::new(DfuStatus::LockUnlocked));
                break;
            }
            if embassy_time::Instant::now() >= deadline {
                info!("dfu_lock: unlock window expired (10 s timeout)");
                DFU_LOCKED.store(true, Ordering::Release);
                publish_event(crate::event::DfuStatusEvent::new(DfuStatus::Idle));
                return;
            }
            embassy_time::Timer::after_millis(50).await;
        }

        info!("dfu_lock: unlocked, waiting for DFU download");
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(10);
        loop {
            if DFU_STARTED.load(Ordering::Acquire) {
                info!("dfu_lock: DFU download started, staying unlocked");
                break;
            }
            if embassy_time::Instant::now() >= deadline {
                info!("dfu_lock: unlock expired (10 s timeout)");
                DFU_LOCKED.store(true, Ordering::Release);
                self.unlocked.store(false, Ordering::Release);
                publish_event(crate::event::DfuStatusEvent::new(DfuStatus::Idle));
                break;
            }
            embassy_time::Timer::after_millis(200).await;
        }
    }
}

#[cfg(feature = "dfu_lock")]
impl<'a> Runnable for DfuLock<'a> {
    async fn run(&mut self) -> ! {
        loop {
            self.process_unlock().await;
        }
    }
}
