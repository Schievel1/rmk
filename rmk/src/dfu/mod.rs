//! # DFU — Device Firmware Update
//!
//! This module implements USB DFU firmware updates for RMK keyboards.
//!
//! ## Data flow
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────────┐
//! │  Host (dfu-util / WebUSB)                                       │
//! │    USB Control Transfer (GET_DESCRIPTOR / DFU_DNLOAD)           │
//! └──────────────┬──────────────────────────────────┬───────────────┘
//!                │                                  │
//!                ▼                                  ▼
//! ┌──────────────────────────┐        ┌──────────────────────────────┐
//! │  UsbDfuIface              │        │  ProxyUsbDfuHandler          │
//! │  (USB control handler)    │        │  (ISR → DFU_CHANNEL)         │
//! │                           │        │                              │
//! │  alt 0 → Central          │        │  target = DfuTarget::Central │
//! │  alt 1 → Peripheral(0)    │        │  writes: DfuCmd::Write(tgt,  │
//! │  alt 2 → Peripheral(1)    │        │          offset, data[512])  │
//! │  ...                      │        └──────────┬───────────────────┘
//! └──────────────────────────┘                   │
//!                                                │
//!                           DFU_CHANNEL (cap 4)
//! ┌───────────────────────────────────────────────────────────────┐
//! │                                                               │
//! │  ┌─── PeripheralManager (central event loop) ──────────────┐ │
//! │  │  peek DFU_CHANNEL for DfuTarget::Peripheral(n)          │ │
//! │  │  forward as SplitMessage::FirmwareChunk → split link    │ │
//! │  │  → peripheral FlashDfuHandler                           │ │
//! │  └─────────────────────────────────────────────────────────┘ │
//! │                                                               │
//! │  ┌─── FlashDfuHandler (central) ──────────────────────────┐ │
//! │  │  peek DFU_CHANNEL for DfuTarget::Central               │ │
//! │  │  start → write_chunk(offset, data[512]) → finish      │ │
//! │  │  erase on demand, NorFlash::write to DFU partition     │ │
//! │  │  finish → sanity check (MSP+reset vector)             │ │
//! │  │         → mark_updated_and_reset()                    │ │
//! │  └─────────────────────────────────────────────────────────┘ │
//! └───────────────────────────────────────────────────────────────┘
//!
//! ┌─── Peripheral (direct calls, no channel) ──────────────────────┐
//! │  FlashDfuHandler::write_chunk(offset, data)                  │
//! │  FlashDfuHandler::compute_dfu_crc() → FirmwareCrcReport        │
//! │  mark_updated_and_reset() only on FirmwareCrcOk                │
//! └─────────────────────────────────────────────────────────────────┘
//!

#[cfg(feature = "dfu_lock")]
use core::sync::atomic::AtomicBool;
#[cfg(feature = "_dfu")]
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

// ---------------------------------------------------------------------------
// Flash partition types and constants
// ---------------------------------------------------------------------------
#[cfg(feature = "_dfu")]
use embassy_boot::FirmwareState;
#[cfg(feature = "_dfu")]
pub use embassy_embedded_hal::flash::partition::Partition;
#[cfg(feature = "_dfu")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "_dfu")]
use embassy_sync::channel::Channel;
#[cfg(any(feature = "dfu_lock", feature = "dfu_split"))]
use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::NorFlash;
#[cfg(feature = "_dfu")]
use heapless;
#[cfg(feature = "_dfu")]
use rmk_types::dfu::DfuStatus;
use static_cell::StaticCell;

#[cfg(feature = "_dfu")]
use crate::core_traits::Runnable;
#[cfg(feature = "_dfu")]
use crate::event::{DfuStatusEvent, publish_event};

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

/// Block size of a DFU download transferred per USB control request.
/// Larger values speed up firmware downloads. Must match the USB control
/// buffer size used by the host.
pub const BLOCK_SIZE_DFU: usize = 512;

/// Partition layout read from the DFU symbols in `memory.x`.
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
/// [`FlashDfuHandler::new`].
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
///
/// This is a free public function because some bootloaders provide DFU with
/// swap functionality outside of RMK. In this case users do not have a
/// `FlashDfuHandler` but they still need to be able to call mark their
/// firmware sucessfully booted.
#[cfg(feature = "_dfu")]
pub async fn mark_booted<STATE: NorFlash>(state: &mut STATE) {
    // 16 bytes is enough for all supported flash types (RP2040: 1, nRF: 4).
    // If a new flash type has WRITE_SIZE > 16, this buffer must be enlarged.
    static ALIGNED: StaticCell<[u8; 16]> = StaticCell::new();
    let mut firmware_state =
        FirmwareState::new(state, &mut ALIGNED.init([0; 16])[..STATE::WRITE_SIZE]);
    firmware_state.mark_booted().await.ok();
}

// ---------------------------------------------------------------------------
// dfu_split sub-module (behind feature flag)
// ---------------------------------------------------------------------------

#[cfg(feature = "dfu_split")]
mod split;
#[cfg(feature = "dfu_split")]
pub use self::split::{
    get_firmware_update_data, read_embedded_firmware_hash, set_firmware_update_data,
};

// ---------------------------------------------------------------------------
// DFU command channel (USB proxy → updater task)
// ---------------------------------------------------------------------------

/// Command queue capacity — USB control block plus slack for split forwarding.
#[cfg(feature = "_dfu")]
const DFU_CMD_QUEUE_SIZE: usize = 4;

/// Identifies the target of a DFU command — local central firmware or a
/// specific split peripheral.
#[cfg(feature = "_dfu")]
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum DfuTarget {
    Central,
    Peripheral(u8),
}

/// A command forwarded from the USB DFU proxy to the async updater task.
#[cfg(feature = "_dfu")]
#[derive(Clone)]
pub(crate) enum DfuCmd {
    Start(DfuTarget),
    /// `Write(target, offset, data)` — offset is the flash byte offset.
    Write(DfuTarget, u32, heapless::Vec<u8, { BLOCK_SIZE_DFU }>),
    Finish(DfuTarget),
    SystemReset(DfuTarget),
}

/// Command channel: the USB DFU proxy (ISR context) sends via
/// [`DFU_CHANNEL`]; the [`FlashDfuHandler`] updater task receives.
#[cfg(feature = "_dfu")]
pub(crate) static DFU_CHANNEL: Channel<CriticalSectionRawMutex, DfuCmd, DFU_CMD_QUEUE_SIZE> = Channel::new();

#[cfg(feature = "_dfu")]
pub(crate) static DFU_WRITE_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Per-peripheral wake signals. The USB ISR ([`ProxyUsbDfuHandler`]) calls
/// `signal(())` after forwarding a command to [`DFU_CHANNEL`]; the matching
/// [`PeripheralManager`](crate::split::driver::PeripheralManager) awaits
/// on `wait()` in its select loop.
#[cfg(feature = "dfu_split")]
pub(crate) static DFU_PERIPH_SIGNALS: [Signal<CriticalSectionRawMutex, ()>; MAX_DFU_ALTS] =
    [const { Signal::new() }; MAX_DFU_ALTS];

// ---------------------------------------------------------------------------
// DFU lock state
// ---------------------------------------------------------------------------

#[cfg(feature = "dfu_lock")]
static DFU_LOCKED: AtomicBool = AtomicBool::new(true);
#[cfg(feature = "dfu_lock")]
static DFU_STARTED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "dfu_lock")]
static DFU_UNLOCK_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Gate shared by the transport's DFU start handlers (central alt 0 and the
/// passthrough slots). Returns `Ok` when a download may proceed and records
/// that it started; while the keys are locked it wakes the unlock state
/// machine and rejects the download with `ErrVendor`.
#[cfg(feature = "_dfu")]
pub(crate) fn dfu_lock_check() -> Result<(), embassy_usb::class::dfu::consts::Status> {
    #[cfg(feature = "dfu_lock")]
    {
        use embassy_usb::class::dfu::consts::Status;
        if DFU_LOCKED.load(Ordering::Acquire) {
            DFU_UNLOCK_SIGNAL.signal(());
            info!("dfu_lock: DFU download rejected — keys not unlocked");
            return Err(Status::ErrVendor);
        }
        DFU_STARTED.store(true, Ordering::Release);
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// FlashDfuHandler — partition-based DFU updater task
// ---------------------------------------------------------------------------

/// Max DFU alternate settings on a single DFU interface.
#[cfg(feature = "_dfu")]
pub(crate) const MAX_DFU_ALTS: usize = 4;

/// Flash-side DFU updater.
///
/// Owns the DFU download and boot state partitions and runs as a [`Runnable`]
/// task on the central. It waits on the command channel ([`DFU_CHANNEL`]) and
/// executes `start`/`write`/`finish`/`system_reset`, fully decoupled from the
/// USB device.
///
/// On the split peripheral, the same struct is used without the [`Runnable`]
/// impl — the event loop calls [`write_chunk`](FlashDfuHandler::write_chunk)
/// and [`compute_dfu_crc`](FlashDfuHandler::compute_dfu_crc) directly.
///
/// The USB side (the proxy in `usb.rs`) never touches flash; all commands flow
/// through the channel. The partitions are typically built with
/// [`partitions_from_linkerscript`]:
///
/// ```
/// let flash_mutex = ::rmk::dfu::FlashMutex::new(flash_driver);
/// let (_, mut state_partition, dfu_partition) =
///     ::rmk::dfu::partitions_from_linkerscript(&flash_mutex);
/// let mut dfu_iface = ::rmk::dfu::FlashDfuHandler::new(dfu_partition, state_partition);
/// ```
#[cfg(feature = "_dfu")]
pub struct FlashDfuHandler<DFU: NorFlash + Clone, STATE: NorFlash + Clone> {
    dfu_partition: DFU,
    state_partition: STATE,
    last_erased_page: Option<u32>,
    written_len: u32,
    offset: u32,
    write_errors: u32,
}

#[cfg(feature = "_dfu")]
impl<DFU: NorFlash + Clone, STATE: NorFlash + Clone> FlashDfuHandler<DFU, STATE> {
    /// Build the DFU updater from a DFU download partition and a boot state
    /// partition.
    pub fn new(dfu_partition: DFU, state_partition: STATE) -> Self {
        Self {
            dfu_partition,
            state_partition,
            last_erased_page: None,
            written_len: 0,
            offset: 0,
            write_errors: 0,
        }
    }

    /// Write a chunk of firmware data at the given partition offset.
    ///
    /// Pages are erased on demand — only the first time a particular page
    /// is encountered. This avoids a long blocking erase of the entire
    /// DFU partition on the very first chunk.
    pub async fn write_chunk(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
        if self.written_len == 0 {
            info!("dfu: firmware update started");
        }
        let mut dfu = self.dfu_partition.clone();
        let erase_size = <DFU as NorFlash>::ERASE_SIZE as u32;
        let start_page = offset / erase_size;
        let end = offset + data.len() as u32;
        let end_page = (end - 1) / erase_size;
        for page in start_page..=end_page {
            if self.last_erased_page != Some(page) {
                dfu.erase(page * erase_size, (page + 1) * erase_size)
                    .await
                    .map_err(|_| ())?;
                self.last_erased_page = Some(page);
            }
        }
        dfu.write(offset, data).await.map_err(|_| ())?;
        self.written_len = self.written_len.max(offset + data.len() as u32);
        publish_event(DfuStatusEvent::new(DfuStatus::Downloading));
        Ok(())
    }

    /// Read back the entire DFU partition and compute its CRC-32.
    ///
    /// Only the bytes up to the highest written offset are included.
    #[cfg(feature = "dfu_split")]
    pub async fn compute_dfu_crc(&self) -> Result<u32, ()> {
        let mut dfu = self.dfu_partition.clone();
        let len = self.written_len as usize;
        let mut crc = crate::crc32::Crc32::new();
        let mut buf = [0u8; 256];
        let mut pos = 0u32;
        while (pos as usize) < len {
            let chunk_len = core::cmp::min(256, len - pos as usize);
            dfu.read(pos, &mut buf[..chunk_len]).await.map_err(|_| ())?;
            crc.update(&buf[..chunk_len]);
            pos += chunk_len as u32;
        }
        Ok(crc.finalize())
    }

    /// Mark the new firmware as valid and reset into it.
    ///
    /// Writes the swap magic bytes to the state partition via embassy-boot's
    /// [`FirmwareState`] and then performs a system reset. The bootloader will
    /// copy the DFU slot to the active slot on the next boot.
    pub async fn mark_updated_and_reset(&mut self) -> Result<(), ()> {
        // 16 bytes is enough for all supported flash types (RP2040: 1, nRF: 4).
        // If a new flash type has WRITE_SIZE > 16, this buffer must be enlarged.
        static ALIGNED: StaticCell<[u8; 16]> = StaticCell::new();
        let mut firmware_state =
            FirmwareState::new(&mut self.state_partition, &mut ALIGNED.init([0; 16])[..STATE::WRITE_SIZE]);
        firmware_state.mark_updated().await.map_err(|_| ())?;
        publish_event(DfuStatusEvent::new(DfuStatus::Finished));
        #[cfg(all(
            target_arch = "arm",
            target_os = "none",
            any(target_abi = "eabi", target_abi = "eabihf")
        ))]
        cortex_m::peripheral::SCB::sys_reset();
        #[allow(unreachable_code)]
        Ok(())
    }

    async fn check_sanity_from_flash(&self) -> Result<(), ()> {
        let mut dfu = self.dfu_partition.clone();
        let mut hdr = [0u8; 8];
        dfu.read(0, &mut hdr).await.map_err(|_| ())?;
        info!("dfu: DFU[0..8] = {:02x}", hdr);
        let all_ff = hdr.iter().all(|&b| b == 0xFF);
        let all_00 = hdr.iter().all(|&b| b == 0x00);
        let msp = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        let reset = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        if all_ff || all_00 || msp == 0 || msp == 0xFFFF_FFFF || reset == 0 || reset == 0xFFFF_FFFF {
            error!(
                "dfu: sanity check failed (msp={:#010x}, reset={:#010x})",
                msp, reset
            );
            return Err(());
        }
        Ok(())
    }
}

#[cfg(feature = "_dfu")]
fn is_central_cmd(cmd: &DfuCmd) -> bool {
    matches!(
        cmd,
        DfuCmd::Start(DfuTarget::Central)
            | DfuCmd::Write(DfuTarget::Central, _, _)
            | DfuCmd::Finish(DfuTarget::Central)
            | DfuCmd::SystemReset(DfuTarget::Central)
    )
}

#[cfg(feature = "_dfu")]
impl<DFU: NorFlash + Clone, STATE: NorFlash + Clone> Runnable for FlashDfuHandler<DFU, STATE> {
    async fn run(&mut self) -> ! {
        loop {
            loop {
                match DFU_CHANNEL.try_peek() {
                    Ok(cmd) if is_central_cmd(&cmd) => {
                        let cmd = DFU_CHANNEL.try_receive().expect("peeked ok");
                        self.handle_cmd(cmd).await;
                    }
                    _ => break,
                }
            }
            embassy_futures::yield_now().await;
        }
    }
}

#[cfg(feature = "_dfu")]
impl<DFU: NorFlash + Clone, STATE: NorFlash + Clone> FlashDfuHandler<DFU, STATE> {
    async fn handle_cmd(&mut self, cmd: DfuCmd) {
        match cmd {
            DfuCmd::Start(DfuTarget::Central) => {
                self.offset = 0;
                self.write_errors = 0;
                DFU_WRITE_ERRORS.store(0, Ordering::Release);
            }
            DfuCmd::Write(DfuTarget::Central, offset, data) => {
                match self.write_chunk(offset, &data).await {
                    Ok(()) => self.offset += data.len() as u32,
                    Err(()) => {
                        error!("dfu: firmware write failed");
                        self.write_errors += 1;
                        self.offset += data.len() as u32;
                        DFU_WRITE_ERRORS.store(self.write_errors, Ordering::Release);
                    }
                }
            }
            DfuCmd::Finish(DfuTarget::Central) => {
                if self.write_errors > 0 {
                    error!("dfu: update aborted - {} write errors occurred", self.write_errors);
                    self.write_errors = 0;
                } else {
                    info!("dfu: {} bytes written, verifying...", self.offset);
                    if self.check_sanity_from_flash().await.is_err() {
                        self.write_errors += 1;
                        DFU_WRITE_ERRORS.store(self.write_errors, Ordering::Release);
                    } else {
                        match self.mark_updated_and_reset().await {
                            Ok(()) => info!("dfu: update complete, resetting"),
                            Err(()) => error!("dfu: firmware finish failed"),
                        }
                    }
                }
            }
            DfuCmd::SystemReset(DfuTarget::Central) => {
                #[cfg(all(
                    target_arch = "arm",
                    target_os = "none",
                    any(target_abi = "eabi", target_abi = "eabihf")
                ))]
                cortex_m::peripheral::SCB::sys_reset();
            }
            // Peripheral commands: skip — handled by the split forwarder
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// dfu_lock
// ---------------------------------------------------------------------------

/// DfuLock state machine that checks a physical key combination to unlock DFU.
#[cfg(feature = "dfu_lock")]
pub struct DfuLock<'a> {
    unlock_keys: &'a [(u8, u8)],
    keymap: &'a crate::keymap::KeyMap<'a>,
}

#[cfg(feature = "dfu_lock")]
impl<'a> DfuLock<'a> {
    pub fn new(unlock_keys: &'a [(u8, u8)], keymap: &'a crate::keymap::KeyMap<'a>) -> Self {
        Self {
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
