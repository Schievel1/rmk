//! A USB DFU runtime interface: `DFU_DETACH` reboots into the bootloader.
//!
//! For firmware whose bootloader does the flashing. The interface carries no
//! data — a host that wants to update the device asks it to leave, then talks
//! DFU to whatever enumerates next. It needs no host protocol, so it works on
//! a dongle that only relays Vial, and with `dfu-util` alone.
//!
//! Leaving for the bootloader is what the host lock exists to gate: a
//! bootloader flashes whatever it is given. With `host_lock` the DETACH is
//! honoured only while the lock stands unlocked; without it a keyboard honours
//! it outright, as its Vial bootloader command already does. A dongle has no
//! keys to unlock with, and being bus-powered it boots when plugged in, so
//! there the DETACH is honoured only in the first seconds after boot. A refused
//! DETACH is simply not acted on; the host sees the device stay.

use embassy_time::Duration;
use embassy_usb::Builder;
use embassy_usb::class::dfu::app_mode::{self, DfuState};
use embassy_usb::class::dfu::consts::DfuAttributes;
use embassy_usb::driver::Driver;
use embassy_usb::msos;
use static_cell::StaticCell;

struct Detach;

impl app_mode::Handler for Detach {
    fn enter_dfu(&mut self) {
        #[cfg(feature = "host_lock")]
        let allowed = crate::host::lock::unlocked();
        #[cfg(all(feature = "dongle", not(feature = "host_lock")))]
        let allowed = embassy_time::Instant::now() <= embassy_time::Instant::from_secs(30);
        #[cfg(not(any(feature = "host_lock", feature = "dongle")))]
        let allowed = true;

        if allowed {
            crate::boot::jump_to_bootloader()
        } else {
            info!("DFU_DETACH refused: locked");
        }
    }
}

pub(crate) fn register<D: Driver<'static>>(builder: &mut Builder<'static, D>) {
    // The vendor interface writes the MS OS 2.0 header when it is present;
    // without it (a Vial-only build) this interface is the first to need one.
    if builder.msos_writer().is_empty() {
        builder.msos_descriptor(msos::windows_version::WIN8_1, super::MSOS_VENDOR_CODE);
    }
    static STATE: StaticCell<DfuState<Detach>> = StaticCell::new();
    // `WILL_DETACH`: the device resets itself on DETACH rather than waiting for
    // the host's bus reset, which neither WebUSB nor nusb on Windows can issue.
    let state = STATE.init(DfuState::new(
        Detach,
        DfuAttributes::CAN_DOWNLOAD | DfuAttributes::WILL_DETACH,
        Duration::from_millis(1000),
    ));
    // WinUSB, so a browser or libusb can send the DETACH on Windows.
    app_mode::usb_dfu(builder, state, |func| {
        func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    });
}
