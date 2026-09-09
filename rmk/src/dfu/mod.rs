//! # DFU — Device Firmware Update
//!
//! This module implements USB DFU firmware updates for RMK keyboards.
//!
//! ## Data flow
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────────┐
//! │  Host (dfu-util / WebUSB)                                        │
//! │    USB Control Transfer (GET_DESCRIPTOR / DFU_DNLOAD)            │
//! └──────────────┬──────────────────────────────────┬────────────-───┘
//!                │                                  │
//!                ▼                                  ▼
//! ┌─-─────────────────────────┐        ┌──────────────────────────────┐
//! │  UsbDfuIface              │        │  ProxyUsbDfuHandler          │
//! │  (USB control handler)    │        │  (ISR → publish_event)       │
//! │                           │        │                              │
//! │  alt 0 → Central          │        │  target = DfuTarget::Central │
//! │  alt 1 → Peripheral(0)    │        │  publish_event(DfuCmdEvent)  │
//! │  alt 2 → Peripheral(1)    │        └──────────┬───────────────────┘
//! │  ...                      │                   │
//! └──-────────────────────────┘                   │
//!                                                 │
//!                          DfuCmdEvent (PubSub)   ▼
//! ┌───────────────────────────────────────────────────────────────┐
//! │                                                               │
//! │  ┌─── PeripheralManager (central event loop) ──────────────┐  │
//! │  │  DfuCmdEvent::subscriber() → filter Peripheral(id)      │  │
//! │  │  forward as SplitMessage::FirmwareChunk → split link    │──────────┐
//! │  │  → peripheral FlashDfuHandler                           │  │       │
//! │  └─────────────────────────────────────────────────────────┘  │       │
//! │                                                               │       │
//! │  ┌─── FlashDfuHandler (central) ─────────────────────────-─┐  │       │
//! │  │  DfuCmdEvent::subscriber() → filter Central             │  │       │
//! │  │  start → write_chunk(offset, data[512]) → finish        │  │       │
//! │  │  erase on demand, NorFlash::write to DFU partition      │  │       │
//! │  │  finish → sanity check (MSP+reset vector)               │  │       │
//! │  │         → mark_updated_and_reset()                      │  │       │
//! │  └──────────────────────────────────────────────────────-──┘  │       │
//! └───────────────────────────────────────────────────────────────┘       │
//!                                                                         │
//!                                                ┌─────split link (UART)──┘
//!                                                ▼                         
//! ┌─── Peripheral (direct calls, no channel) ─────────────────────┐       
//! │  FlashDfuHandler::write_chunk(offset, data)                   │       
//! │  FlashDfuHandler::compute_dfu_crc() → FirmwareCrcReport       │       
//! │  mark_updated_and_reset() only on FirmwareCrcOk               │       
//! └───────────────────────────────────────────────────────────────┘
//!

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_boot::FirmwareState;
pub use embassy_embedded_hal::flash::partition::Partition;
#[cfg(feature = "dfu_lock")]
use embassy_futures::select::{Either, select};
use embedded_storage_async::nor_flash::NorFlash;
use heapless;
use rmk_types::dfu::DfuStatus;

use crate::core_traits::Runnable;
use crate::event::{DfuCmdEvent, DfuStatusEvent, SubscribableEvent, publish_event};

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

/// Block size of a DFU download transferred per USB control request.
/// Larger values speed up firmware downloads. Must match the USB control
/// buffer size used by the host.
pub const BLOCK_SIZE_DFU: usize = 512;

/// Partition layout read from the DFU symbols in `memory.x`.
/// The offsets are flash-relative and come from the `__bootloader_*` symbols
/// that rmk-boot's generated `memory.x` provides.
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
pub type FlashMutex<F> = embassy_sync::mutex::Mutex<crate::RawMutex, F>;

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
pub fn partitions_from_linkerscript<'a, F: NorFlash>(
    flash_mutex: &'a FlashMutex<F>,
) -> (
    Partition<'a, crate::RawMutex, F>,
    Partition<'a, crate::RawMutex, F>,
    Partition<'a, crate::RawMutex, F>,
) {
    let layout = dfu_flash_layout();
    let storage = Partition::new(flash_mutex, layout.storage_offset, layout.storage_size);
    let state = Partition::new(flash_mutex, layout.state_offset, layout.state_size);
    let dfu = Partition::new(flash_mutex, layout.dfu_offset, layout.dfu_size);
    (storage, state, dfu)
}

/// Mark firmware boot as successful so the bootloader doesn't revert the
/// update on the next reset.
///
/// `state` is the boot state partition — typically built with
/// [`partitions_from_linkerscript`].
///
/// Must be called *after* the firmware is confirmed running.
///
/// When using [`FlashDfuHandler`], this is called automatically — but users
/// need this free function if they use a swap-based bootloader outside of RMK
/// and don't have a `FlashDfuHandler`.
pub async fn mark_booted<STATE: NorFlash>(state: &mut STATE) {
    let mut aligned = [0u8; 16];
    let mut firmware_state = FirmwareState::new(state, &mut aligned[..STATE::WRITE_SIZE]);
    firmware_state.mark_booted().await.ok();
}

#[cfg(feature = "dfu_split")]
mod split;
#[cfg(feature = "dfu_split")]
pub use self::split::{get_firmware_update_data, read_embedded_firmware_hash, set_firmware_update_data};

/// Identifies the target of a DFU command — local central firmware or a
/// specific split peripheral.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum DfuTarget {
    Central,
    Peripheral(u8),
}

/// A command forwarded from the USB DFU proxy to the async updater task.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum DfuCmd {
    Start(DfuTarget),
    /// `Write(target, offset, data)` — offset is the flash byte offset.
    Write(DfuTarget, u32, heapless::Vec<u8, { BLOCK_SIZE_DFU }>),
    Finish(DfuTarget),
    SystemReset(DfuTarget),
    /// Wake-up signal from `dfu_lock_check()` to the `DfuLock` unlock state machine.
    UnlockRequest,
}

impl DfuCmd {
    /// The target (central or peripheral) of this command, if applicable.
    pub(crate) fn target(&self) -> Option<DfuTarget> {
        match self {
            DfuCmd::Start(t) | DfuCmd::Write(t, _, _) | DfuCmd::Finish(t) | DfuCmd::SystemReset(t) => Some(*t),
            DfuCmd::UnlockRequest => None,
        }
    }

    /// Returns `true` when this command is for the central flash updater.
    pub(crate) fn is_central(&self) -> bool {
        matches!(self.target(), Some(DfuTarget::Central))
    }

    /// Returns `true` when this command targets a known peripheral (valid id range).
    pub(crate) fn is_valid_peripheral(&self) -> bool {
        matches!(self.target(), Some(DfuTarget::Peripheral(id)) if (id as usize) < MAX_DFU_ALTS)
    }
}

/// Set to `true` by the [`FlashDfuHandler`] when a flash write fails.
/// The USB proxy reads this flag to reject subsequent `DFU_DNLOAD` / `DFU_UPLOAD`
/// requests with `ERR_WRITE` instead of forwarding corrupted data.
pub(crate) static DFU_WRITE_FAILED: AtomicBool = AtomicBool::new(false);

/// Gate shared by the transport's DFU start handlers (central alt 0 and the
/// passthrough slots). Returns `Ok` when a download may proceed; while the
/// keys are locked it wakes the unlock state machine and rejects the download
/// with `ErrVendor`.
pub(crate) fn dfu_lock_check() -> Result<(), embassy_usb::class::dfu::consts::Status> {
    #[cfg(feature = "dfu_lock")]
    {
        use embassy_usb::class::dfu::consts::Status;
        if DFU_LOCKED.load(Ordering::Acquire) {
            publish_event(DfuCmdEvent(DfuCmd::UnlockRequest));
            info!("dfu_lock: DFU download rejected — keys not unlocked");
            return Err(Status::ErrVendor);
        }
    }
    Ok(())
}

/// Max DFU alternate settings on a single DFU interface.
/// Alt 0 is always the central; the remaining slots are split peripherals.
pub(crate) const MAX_DFU_ALTS: usize = crate::SPLIT_PERIPHERALS_NUM + 1;

/// Flash-side DFU updater.
///
/// Owns the DFU download and boot state partitions and runs as a [`Runnable`]
/// task on the central. It subscribes to [`DfuCmdEvent`] and executes
/// `start`/`write`/`finish`/`system_reset`, fully decoupled from the
/// USB device.
///
/// On the split peripheral, the same struct is used without the [`Runnable`]
/// impl — the event loop calls [`write_chunk`](FlashDfuHandler::write_chunk)
/// and [`compute_dfu_crc`](FlashDfuHandler::compute_dfu_crc) directly.
///
/// The USB side (the proxy in `usb.rs`) never touches flash; all commands flow
/// through the event system. The partitions are typically built with
/// [`partitions_from_linkerscript`]:
///
/// ```ignore
/// let flash_mutex = ::rmk::dfu::FlashMutex::new(flash_driver);
/// let (_, mut state_partition, dfu_partition) =
///     ::rmk::dfu::partitions_from_linkerscript(&flash_mutex);
/// let mut dfu_iface = ::rmk::dfu::FlashDfuHandler::new(dfu_partition, state_partition);
/// ```
pub struct FlashDfuHandler<DFU: NorFlash + Clone, STATE: NorFlash + Clone> {
    dfu_partition: DFU,
    state_partition: STATE,
    last_erased_page: Option<u32>,
    written_len: u32,
    offset: u32,
}

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
        }
    }

    /// Write a chunk of firmware data at the given partition offset.
    ///
    /// Pages are erased on demand — only the first time a particular page
    /// is encountered. This avoids a long blocking erase of the entire
    /// DFU partition on the very first chunk.
    pub async fn write_chunk(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
        if offset == 0 {
            // New download session: drop stale state from an aborted previous
            // session so its tail can't poison this image's CRC and its erase
            // cache can't skip erasing page 0. The central always streams
            // monotonically from 0; chunk-0 retries happen before any other
            // offset is written, so this is safe.
            self.written_len = 0;
            self.last_erased_page = None;
        }
        // Log once per session so the user sees progress.
        if self.written_len == 0 {
            info!("dfu: firmware update started");
        }
        let mut dfu = self.dfu_partition.clone();
        if data.is_empty() {
            return Ok(());
        }
        // Calculate which flash pages overlap with [offset .. offset+len)
        // and erase each one exactly once.
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
        // Write the actual firmware bytes.
        dfu.write(offset, data).await.map_err(|_| ())?;
        // Track the highest written offset (used by compute_dfu_crc).
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
        let mut aligned = [0u8; 16];
        let mut firmware_state = FirmwareState::new(&mut self.state_partition, &mut aligned[..STATE::WRITE_SIZE]);
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

    /// Mark firmware boot as successful so the bootloader doesn't revert the
    /// update on the next reset.
    ///
    /// Called automatically on the first iteration of [`Runnable::run`] (central)
    /// and in [`run_rmk_split_peripheral`](crate::split::peripheral::run_rmk_split_peripheral)
    /// (peripheral).
    pub async fn mark_booted(&mut self) {
        mark_booted(&mut self.state_partition).await;
    }

    /// Read the first 8 bytes of the DFU partition and verify they look like
    /// a valid Cortex-M vector table (non-blank, non-erased MSP and reset
    /// handler).  Returns `Err` when the image is obviously corrupt — the
    /// caller aborts the update instead of resetting into a bricked firmware.
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
            error!("dfu: sanity check failed (msp={:#010x}, reset={:#010x})", msp, reset);
            return Err(());
        }
        Ok(())
    }
}

impl<DFU: NorFlash + Clone, STATE: NorFlash + Clone> Runnable for FlashDfuHandler<DFU, STATE> {
    async fn run(&mut self) -> ! {
        self.mark_booted().await;
        let mut sub = DfuCmdEvent::subscriber();
        loop {
            let cmd_event = sub.next_message_pure().await;
            if cmd_event.0.is_central() {
                self.handle_cmd(cmd_event.0).await;
                continue;
            }
            #[cfg(feature = "dfu_split")]
            if !cmd_event.0.is_valid_peripheral() {
                warn!("dfu: dropping orphaned command for unknown peripheral target");
                continue;
            }
            // Valid peripheral command — handled by PeripheralManager.
        }
    }
}

impl<DFU: NorFlash + Clone, STATE: NorFlash + Clone> FlashDfuHandler<DFU, STATE> {
    async fn handle_cmd(&mut self, cmd: DfuCmd) {
        match cmd {
            DfuCmd::UnlockRequest => {}
            DfuCmd::Start(DfuTarget::Central) => {
                self.offset = 0;
                DFU_WRITE_FAILED.store(false, Ordering::Release);
            }
            DfuCmd::Write(DfuTarget::Central, offset, data) => match self.write_chunk(offset, &data).await {
                Ok(()) => self.offset = offset + data.len() as u32,
                Err(()) => {
                    error!("dfu: firmware write failed at offset {:#010x}", offset);
                    self.offset = offset + data.len() as u32;
                    DFU_WRITE_FAILED.store(true, Ordering::Release);
                    publish_event(DfuStatusEvent::new(DfuStatus::Error));
                }
            },
            DfuCmd::Finish(DfuTarget::Central) => {
                if DFU_WRITE_FAILED.load(Ordering::Acquire) {
                    error!("dfu: update aborted - write errors occurred");
                    publish_event(DfuStatusEvent::new(DfuStatus::Error));
                    DFU_WRITE_FAILED.store(false, Ordering::Release);
                } else {
                    info!("dfu: {} bytes written, verifying...", self.offset);
                    if self.check_sanity_from_flash().await.is_err() {
                        DFU_WRITE_FAILED.store(true, Ordering::Release);
                        publish_event(DfuStatusEvent::new(DfuStatus::Error));
                    } else {
                        info!("dfu: looks good, restarting");
                        match self.mark_updated_and_reset().await {
                            Ok(()) => info!("dfu: update complete, resetting"),
                            Err(()) => {
                                error!("dfu: firmware finish failed");
                                publish_event(DfuStatusEvent::new(DfuStatus::Error));
                            }
                        }
                    }
                }
            }
            DfuCmd::SystemReset(DfuTarget::Central) => {
                crate::boot::reboot_keyboard();
            }
            // Peripheral commands: skip — handled by the split forwarder
            _ => {}
        }
    }
}

/// `true` while DFU is locked (default). Cleared by `DfuLock` when unlock keys are pressed.
#[cfg(feature = "dfu_lock")]
static DFU_LOCKED: AtomicBool = AtomicBool::new(true);

/// Physical-key gate that prevents accidental or unauthorised DFU downloads.
///
/// When enabled via the `dfu_lock` feature, every DFU download attempt
/// (central or peripheral passthrough) first triggers a
/// [`DfuCmd::UnlockRequest`](crate::dfu::DfuCmd::UnlockRequest) event.
/// [`DfuLock`] picks up that event, opens a 10 s unlock window, and polls
/// the matrix at 50 ms intervals looking for the key combination configured
/// in `unlock_keys`.  Once all listed keys are pressed simultaneously the
/// global [`DFU_LOCKED`] flag is cleared and another 10 s countdown starts —
/// the host must begin the actual download within that window, otherwise the
/// lock engages again.
///
/// # Example
///
/// ```ignore
/// let mut dfu_lock = DfuLock::new(&[(0, 0), (1, 1)], &keymap);
/// // in run_all!:
/// dfu_lock,
/// ```
#[cfg(feature = "dfu_lock")]
pub struct DfuLock<'a> {
    unlock_keys: &'a [(u8, u8)],
    keymap: &'a crate::keymap::KeyMap<'a>,
}

#[cfg(feature = "dfu_lock")]
impl<'a> DfuLock<'a> {
    /// Create a new DFU lock.
    ///
    /// `unlock_keys` lists matrix positions `(row, col)` that must all be
    /// held down simultaneously to unlock DFU.  `keymap` is used to read
    /// the physical matrix state.
    pub fn new(unlock_keys: &'a [(u8, u8)], keymap: &'a crate::keymap::KeyMap<'a>) -> Self {
        Self { unlock_keys, keymap }
    }
}

#[cfg(feature = "dfu_lock")]
impl<'a> Runnable for DfuLock<'a> {
    /// Waits for a DFU download attempt, polls the matrix for the unlock
    /// key combination, then monitors the download until completion or stall.
    /// Re-locks automatically on every exit path.
    async fn run(&mut self) -> ! {
        let mut dfu_cmd_sub = DfuCmdEvent::subscriber();

        loop {
            // 1. Warte auf UnlockRequest
            match dfu_cmd_sub.next_message_pure().await {
                DfuCmdEvent(DfuCmd::UnlockRequest) => {}
                _ => continue,
            }

            info!("dfu_lock: DFU activity detected, unlock window open for 10 s");
            info!("dfu_lock: waiting for unlock keys");
            publish_event(DfuStatusEvent::new(DfuStatus::LockWaiting));

            // 2. Unlock-Key-Polling (10 s)
            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(10);
            let mut unlocked = false;
            loop {
                let all_pressed = self
                    .unlock_keys
                    .iter()
                    .all(|(row, col)| self.keymap.read_matrix_key(*row, *col));
                if all_pressed {
                    DFU_LOCKED.store(false, Ordering::Release);
                    info!("dfu_lock: unlock keys pressed, DFU unlocked for 10 s");
                    publish_event(DfuStatusEvent::new(DfuStatus::LockUnlocked));
                    unlocked = true;
                    break;
                }
                if embassy_time::Instant::now() >= deadline {
                    info!("dfu_lock: unlock window expired (10 s timeout)");
                    publish_event(DfuStatusEvent::new(DfuStatus::Idle));
                    break;
                }
                embassy_time::Timer::after_millis(50).await;
            }
            if !unlocked {
                continue;
            }

            // 3. Warte auf Start oder Timeout (10 s)
            info!("dfu_lock: unlocked, waiting for DFU download");
            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(10);
            match select(dfu_cmd_sub.next_message_pure(), embassy_time::Timer::at(deadline)).await {
                Either::First(DfuCmdEvent(DfuCmd::Start(_))) => {
                    info!("dfu_lock: DFU download started");
                }
                _ => {
                    info!("dfu_lock: unlock expired, no download started");
                    DFU_LOCKED.store(true, Ordering::Release);
                    publish_event(DfuStatusEvent::new(DfuStatus::Idle));
                    continue;
                }
            }

            // 4. Activity-basiertes Warten auf Finish
            let mut write_deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(30);
            loop {
                match select(dfu_cmd_sub.next_message_pure(), embassy_time::Timer::at(write_deadline)).await {
                    Either::First(DfuCmdEvent(DfuCmd::Write(..))) => {
                        write_deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(30);
                    }
                    Either::First(DfuCmdEvent(DfuCmd::Finish(_))) => {
                        info!("dfu_lock: download complete, re-locking");
                        break;
                    }
                    Either::First(DfuCmdEvent(DfuCmd::SystemReset(_))) => {
                        info!("dfu_lock: reset requested, re-locking");
                        break;
                    }
                    _ => {
                        info!("dfu_lock: download stalled (30 s no activity), re-locking");
                        break;
                    }
                }
            }
            DFU_LOCKED.store(true, Ordering::Release);
            publish_event(DfuStatusEvent::new(DfuStatus::Idle));
        }
    }
}
