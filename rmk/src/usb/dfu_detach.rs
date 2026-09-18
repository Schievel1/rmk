//! A USB DFU runtime interface: `DFU_DETACH` reboots into the bootloader.
//!
//! For firmware whose bootloader does the flashing (rmk-boot, Adafruit, or a
//! single-bank bootloader on a part too small for two slots). The interface
//! carries no data — a host that wants to update the device asks it to leave,
//! then talks DFU to whatever enumerates next. It needs no host protocol, so it
//! works on a dongle that only relays Vial, and with `dfu-util` alone.
//!
//! Leaving for the bootloader is what the host lock exists to gate: a
//! bootloader flashes whatever it is given. With a host lock the DETACH is
//! honoured only while it stands unlocked, the same proof of presence the Rynk
//! bootloader command demands. Without one — a dongle, which has no keys, or a
//! Vial-only build — the one physical act left is plugging the device in, so
//! the DETACH is honoured only in the seconds after the bus comes up.

use embassy_time::Duration;
#[cfg(not(feature = "host_lock"))]
use embassy_time::Instant;
use embassy_usb::Builder;
use embassy_usb::class::dfu::app_mode::{self, DfuState};
use embassy_usb::class::dfu::consts::DfuAttributes;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::Driver;
use embassy_usb::{Handler, msos};
use static_cell::StaticCell;

/// `bRequest` of `DFU_DETACH`.
const REQ_DETACH: u8 = 0;
/// DFU functional descriptor type, and the class triple of a runtime interface.
const DESC_DFU_FUNCTIONAL: u8 = 0x21;
const CLASS_APPLICATION_SPECIFIC: u8 = 0xFE;
const SUBCLASS_DFU: u8 = 0x01;
const PROTOCOL_RUNTIME: u8 = 0x01;

/// `wDetachTimeout`: moot with `WILL_DETACH`, but the descriptor needs one.
const DETACH_TIMEOUT: Duration = Duration::from_millis(1000);

/// How long after the bus comes up a device without a host lock honours a
/// DETACH. Long enough to replug and click; short enough that a host cannot
/// simply wait for it.
#[cfg(not(feature = "host_lock"))]
const PLUG_WINDOW: Duration = Duration::from_secs(30);

struct Detach;

impl app_mode::Handler for Detach {
    fn enter_dfu(&mut self) {
        crate::boot::jump_to_bootloader()
    }
}

/// The class state machine behind the presence gate.
struct Gate {
    inner: DfuState<Detach>,
    /// When the bus last came up.
    #[cfg(not(feature = "host_lock"))]
    plugged: Option<Instant>,
}

impl Gate {
    fn allowed(&self) -> bool {
        #[cfg(feature = "host_lock")]
        {
            crate::host::lock::unlocked()
        }
        #[cfg(not(feature = "host_lock"))]
        {
            self.plugged.is_some_and(|at| at.elapsed() <= PLUG_WINDOW)
        }
    }
}

impl Handler for Gate {
    #[cfg(not(feature = "host_lock"))]
    fn enabled(&mut self, enabled: bool) {
        if enabled {
            self.plugged = Some(Instant::now());
        }
    }

    fn reset(&mut self) {
        self.inner.reset()
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        let detach =
            (req.request_type, req.recipient, req.request) == (RequestType::Class, Recipient::Interface, REQ_DETACH);
        if detach && !self.allowed() {
            info!("DFU_DETACH refused: locked");
            return Some(OutResponse::Rejected);
        }
        self.inner.control_out(req, data)
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        self.inner.control_in(req, buf)
    }
}

pub(crate) fn register<D: Driver<'static>>(builder: &mut Builder<'static, D>) {
    // The vendor interface writes the MS OS 2.0 header when it is present;
    // without it (a Vial-only build) this interface is the first to need one.
    if builder.msos_writer().is_empty() {
        builder.msos_descriptor(msos::windows_version::WIN8_1, super::MSOS_VENDOR_CODE);
    }

    // `WILL_DETACH`: the device resets itself on DETACH rather than waiting for
    // the host's bus reset, which neither WebUSB nor nusb on Windows can issue.
    let attrs = DfuAttributes::CAN_DOWNLOAD | DfuAttributes::WILL_DETACH;
    let timeout_ms = DETACH_TIMEOUT.as_millis() as u16;

    let mut func = builder.function(0x00, 0x00, 0x00);
    // WinUSB, so a browser or libusb can send the DETACH on Windows.
    func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    let mut iface = func.interface();
    let mut alt = iface.alt_setting(CLASS_APPLICATION_SPECIFIC, SUBCLASS_DFU, PROTOCOL_RUNTIME, None);
    alt.descriptor(
        DESC_DFU_FUNCTIONAL,
        &[
            attrs.bits(),
            (timeout_ms & 0xff) as u8,
            (timeout_ms >> 8) as u8,
            // wTransferSize: the bootloader's own descriptor governs the download.
            0x40,
            0x00,
            // bcdDFUVersion 1.1
            0x10,
            0x01,
        ],
    );
    drop(func);

    static GATE: StaticCell<Gate> = StaticCell::new();
    builder.handler(GATE.init(Gate {
        inner: DfuState::new(Detach, attrs, DETACH_TIMEOUT),
        #[cfg(not(feature = "host_lock"))]
        plugged: None,
    }));
}
