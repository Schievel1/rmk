//! A USB DFU runtime interface: `DFU_DETACH` reboots into the bootloader.
//!
//! For firmware whose bootloader does the flashing (rmk-boot, Adafruit, or a
//! single-bank bootloader on a part too small for two slots). The interface
//! carries no data — a host that wants to update the device asks it to leave,
//! then talks DFU to whatever enumerates next. It needs no host protocol, so it
//! works on a dongle that only relays Vial, and with `dfu-util` alone.

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
        crate::boot::jump_to_bootloader()
    }
}

pub(crate) fn register<D: Driver<'static>>(builder: &mut Builder<'static, D>) {
    static STATE: StaticCell<DfuState<Detach>> = StaticCell::new();
    // `WILL_DETACH`: the device resets itself on DETACH rather than waiting for
    // the host's bus reset, which WebUSB cannot always issue.
    let state = STATE.init(DfuState::new(
        Detach,
        DfuAttributes::CAN_DOWNLOAD | DfuAttributes::WILL_DETACH,
        Duration::from_millis(1000),
    ));
    // The vendor interface writes the MS OS 2.0 header when it is present;
    // without it (a Vial-only build) this interface is the first to need one.
    if builder.msos_writer().is_empty() {
        builder.msos_descriptor(msos::windows_version::WIN8_1, super::MSOS_VENDOR_CODE);
    }
    app_mode::usb_dfu(builder, state, |func| {
        // WinUSB, so a browser or libusb can send the DETACH on Windows.
        func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    });
}
