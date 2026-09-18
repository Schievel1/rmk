//! A USB DFU runtime interface for dongles: `DFU_DETACH` reboots into the
//! bootloader.
//!
//! A dongle relays its keyboard's host protocol, so the Rynk and Vial
//! bootloader commands never reach it; and with one flash slot it cannot take
//! a download itself. This interface carries no data — a host that wants to
//! update the dongle asks it to leave, then talks DFU to the bootloader that
//! enumerates next. `dfu-util` and rmk-gui both speak it.
//!
//! A bootloader flashes whatever it is given, so leaving for it needs proof
//! that someone is there. A dongle has no keys, but being bus-powered it boots
//! when plugged in: the DETACH is honoured only in the first seconds after
//! boot. Later ones are simply not acted on; the host sees the dongle stay.

use embassy_time::{Duration, Instant};
use embassy_usb::Builder;
use embassy_usb::class::dfu::app_mode::{self, DfuState};
use embassy_usb::class::dfu::consts::DfuAttributes;
use embassy_usb::driver::Driver;
use embassy_usb::msos;
use static_cell::StaticCell;

#[cfg(not(feature = "dongle"))]
compile_error!("`dfu_detach` is for dongles: a keyboard enters its bootloader through its host protocol");

/// Long enough to replug and click; short enough that a host cannot simply
/// wait for it.
const PLUG_WINDOW: Duration = Duration::from_secs(30);

struct Detach;

impl app_mode::Handler for Detach {
    fn enter_dfu(&mut self) {
        if Instant::now() <= Instant::MIN + PLUG_WINDOW {
            crate::boot::jump_to_bootloader()
        } else {
            info!("DFU_DETACH ignored: plug-in window over");
        }
    }
}

pub(crate) fn register<D: Driver<'static>>(builder: &mut Builder<'static, D>) {
    // The vendor interface writes the MS OS 2.0 header when it is present;
    // without it (a Vial dongle) this interface is the first to need one.
    if builder.msos_writer().is_empty() {
        builder.msos_descriptor(msos::windows_version::WIN8_1, super::MSOS_VENDOR_CODE);
    }
    static STATE: StaticCell<DfuState<Detach>> = StaticCell::new();
    // `WILL_DETACH`: the dongle resets itself on DETACH rather than waiting for
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
