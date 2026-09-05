use core::sync::atomic::Ordering;

#[cfg(feature = "dfu_nrf")]
use embassy_boot::{FirmwareUpdater, FirmwareUpdaterConfig};
#[cfg(feature = "dfu_rp")]
use embassy_boot_rp::{FirmwareUpdater, FirmwareUpdaterConfig};
#[cfg(feature = "dfu_nrf")]
use embassy_nrf::nvmc::Nvmc;
#[cfg(feature = "dfu_rp")]
use embassy_rp::flash::{Blocking, Flash};
#[cfg(feature = "dfu_rp")]
use embassy_rp::peripherals::FLASH;
use embedded_storage_async::nor_flash::NorFlash;
use rmk_types::dfu::DfuStatus;
use static_cell::StaticCell;

use crate::event::{DfuStatusEvent, publish_event};

/// The DFU/state partition type over the build's internal flash.
#[cfg(feature = "dfu_rp")]
pub type InternalFlashPartition = crate::dfu::DfuPartition<
    'static,
    embassy_embedded_hal::adapter::BlockingAsync<Flash<'static, FLASH, Blocking, { super::super::FLASH_SIZE }>>,
>;
#[cfg(feature = "dfu_nrf")]
pub type InternalFlashPartition =
    crate::dfu::DfuPartition<'static, embassy_embedded_hal::adapter::BlockingAsync<Nvmc<'static>>>;

// =========================================================================
// SplitDfuHandler — peripheral-side firmware writing
// =========================================================================

/// Handles firmware chunk writes on the **peripheral** side of a split
/// keyboard during an over-the-split-link DFU update.
///
/// Created when the first [`SplitMessage::FirmwareChunk`] arrives. Flash
/// pages are erased incrementally (only when a new page boundary is hit), so
/// the first chunk does not stall the split link for the full erase time.
///
/// The partitions are passed in by value by the split peripheral driver
/// (typically the DFU download and boot state partitions over the
/// peripheral's internal flash).
///
/// # Lifecycle
///
/// 1. [`SplitDfuHandler::new`] — take ownership of the partitions.
/// 2. [`write_chunk`](SplitDfuHandler::write_chunk) — erase + write.
/// 3. [`compute_dfu_crc`](SplitDfuHandler::compute_dfu_crc) — read
///    back the entire DFU partition and return its CRC-32.
/// 4. [`mark_updated_and_reset`](SplitDfuHandler::mark_updated_and_reset)
///    — tell embassy-boot the new firmware is valid, then reset into it.
pub struct SplitDfuHandler<DFU: NorFlash + Clone, STATE: NorFlash + Clone> {
    dfu_partition: DFU,
    state_partition: STATE,
    last_erased_page: Option<u32>,
    written_len: u32,
}

impl<DFU: NorFlash + Clone, STATE: NorFlash + Clone> SplitDfuHandler<DFU, STATE> {
    /// Create a new handler from a DFU download partition and a boot state
    /// partition.
    pub fn new(dfu_partition: DFU, state_partition: STATE) -> Self {
        if dfu_partition.capacity() == 0 {
            error!("dfu_split: DFU partition size is 0");
        }
        Self {
            dfu_partition,
            state_partition,
            last_erased_page: None,
            written_len: 0,
        }
    }

    /// Write a chunk of firmware data at the given partition offset.
    ///
    /// Pages are erased on demand — only the first time a particular page
    /// is encountered.  This avoids a long blocking erase of the entire
    /// DFU partition on the very first chunk.
    pub async fn write_chunk(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
        if self.written_len == 0 {
            info!("dfu_split: firmware update started");
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
    /// Called by the peripheral during end-to-end verification before
    /// resetting into the new firmware.  Only the bytes up to the
    /// highest written offset are included.
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
    /// Calls `embassy-boot`'s async `mark_updated` and then performs a
    /// system reset.  The bootloader will copy the DFU slot to the
    /// active slot on the next boot.
    pub async fn mark_updated_and_reset(&self) -> Result<(), ()> {
        let mut dfu = self.dfu_partition.clone();
        let mut hdr = [0u8; 8];
        dfu.read(0, &mut hdr).await.map_err(|_| ())?;
        info!("dfu_split: DFU[0..8] = {:02x}", hdr);
        let all_ff = hdr.iter().all(|&b| b == 0xFF);
        let all_00 = hdr.iter().all(|&b| b == 0x00);
        let msp = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        let reset = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        if all_ff || all_00 || msp == 0 || msp == 0xFFFF_FFFF || reset == 0 || reset == 0xFFFF_FFFF {
            error!(
                "dfu_split: sanity check failed (msp={:#010x}, reset={:#010x}), aborting",
                msp, reset
            );
            return Err(());
        }
        let config = FirmwareUpdaterConfig {
            dfu: self.dfu_partition.clone(),
            state: self.state_partition.clone(),
        };
        static ALIGNED: StaticCell<[u8; crate::dfu::DFU_WRITE_SIZE]> = StaticCell::new();
        let aligned: &'static mut [u8] =
            &mut ALIGNED.init([0; crate::dfu::DFU_WRITE_SIZE])[..<DFU as NorFlash>::WRITE_SIZE];
        let mut updater = FirmwareUpdater::new(config, aligned);
        updater.mark_updated().await.map_err(|_| ())?;
        publish_event(DfuStatusEvent::new(DfuStatus::Finished));
        cortex_m::peripheral::SCB::sys_reset()
    }
}

/// Return the CRC-32 of the currently running firmware binary.
///
/// The result is computed once and cached.  The firmware region is
/// determined by the `__vector_table` / `__veneer_limit` linker symbols,
/// covering the entire `.text` + `.rodata` + `.data` sections.
pub fn read_embedded_firmware_hash() -> u32 {
    use core::sync::atomic::AtomicU32;
    static CACHED_HASH: AtomicU32 = AtomicU32::new(0);
    let cached = CACHED_HASH.load(Ordering::Acquire);
    if cached != 0 {
        return cached;
    }
    unsafe extern "C" {
        static __vector_table: u8;
        static __veneer_limit: u8;
    }
    let start = unsafe { &__vector_table as *const u8 };
    let end = unsafe { &__veneer_limit as *const u8 };
    let len = end as usize - start as usize;
    let data = unsafe { core::slice::from_raw_parts(start, len) };
    let hash = crate::crc32::crc32(data);
    CACHED_HASH.store(hash, Ordering::Release);
    hash
}
