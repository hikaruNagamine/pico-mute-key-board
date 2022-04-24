#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use core::cell::RefCell;
use cortex_m::delay::Delay;
use cortex_m::interrupt::CriticalSection;
use cortex_m::interrupt::Mutex;
use once_cell::unsync::OnceCell;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
use rp_pico::hal;
use rp_pico::hal::pac::interrupt;

// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::DynPin,
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
};

// USB Device support
use usb_device as usbd;
// use usbd::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
// use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
// use usbd_hid::descriptor::MouseReport;
use usbd_hid::descriptor::SerializedDescriptor;
// use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<usbd::prelude::UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<usbd::class_prelude::UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
// static mut USB_HID: Option<usbd_hid::hid_class::HIDClass<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID_KEYBOARD: Option<usbd_hid::hid_class::HIDClass<hal::usb::UsbBus>> = None;

static DELAY: Mutex<OnceCell<RefCell<Delay>>> = Mutex::new(OnceCell::new());
#[doc(hidden)]
fn delay_ms(ms: u32) {
    // debug!("delay_ms {}", ms);
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        let cs = CriticalSection::new();
        let delay = &mut *DELAY.borrow(&cs).get().unwrap().borrow_mut();
        delay.delay_ms(ms);
    });
}

#[entry]
fn main() -> ! {
    info!("Program start");
    match body() {
        Ok(_) => {}
        Err(e) => {
            error!("ERROR : ");
        }
    }
    loop {
        cortex_m::asm::wfi();
    }
}
fn body() -> Result<(), usb_device::UsbError> {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // usb setting
    let usb_bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    let bus_allocator = usbd::class_prelude::UsbBusAllocator::new(usb_bus);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(bus_allocator);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    // let usb_hid = usbd_hid::hid_class::HIDClass::new(bus_ref, MouseReport::desc(), 5);
    // let usb_hid = usbd_hid::hid_class::HIDClass::new(bus_ref, KeyboardReport::desc(), 60);
    let usb_hid_keyboard = usbd_hid::hid_class::HIDClass::new(bus_ref, KeyboardReport::desc(), 10);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        // USB_HID = Some(usb_hid);
        USB_HID_KEYBOARD = Some(usb_hid_keyboard);
    }

    let vid_pid = usbd::device::UsbVidPid(0x3333, 0x0333);
    let usb_dev = usbd::device::UsbDeviceBuilder::new(bus_ref, vid_pid)
        .manufacturer("mitsumaru company")
        .product("pico-mute-key-board")
        .serial_number("TEST")
        .build();
    // https://www.usb.org/defined-class-codes#anchor_BaseClassEFh
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    unsafe {
        let cs = CriticalSection::new();
        let global_delay = DELAY.borrow(&cs);
        global_delay.set(RefCell::new(delay)).map_err(|_| 0);
    }
    // gpio setting
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // let mut led_pin = pins.led.into_push_pull_output();
    // output
    let row1 = pins.gpio2.into_push_pull_output();
    let row2 = pins.gpio4.into_push_pull_output();
    let row3 = pins.gpio3.into_push_pull_output();
    // input
    let col1 = pins.gpio19.into_pull_down_input();
    let col2 = pins.gpio18.into_pull_down_input();

    let rows: [&mut DynPin; 3] = [&mut row1.into(), &mut row2.into(), &mut row3.into()];
    let cols: [&mut DynPin; 2] = [&mut col1.into(), &mut col2.into()];

    let mut key_scanner = KeyScanner::new(rows, cols);
    let mut last_keycodes = [0u8; 6];
    info!("loop start");
    loop {
        delay_ms(10);

        // let key_repo = KeyboardReport {
        //     modifier: 0,
        //     reserved: 0,
        //     leds: 0,
        //     keycodes: [0x04 /* A */, 0, 0, 0, 0, 0],
        // };
        let key_repo = key_scanner.scan();
        if last_keycodes != key_repo.keycodes {
            push_key_event(key_repo).ok().unwrap_or(0);
            last_keycodes = key_repo.keycodes;
        }

        // delay_ms(10);

        // debug!("mouse up");
        // let rep_up = MouseReport {
        //     x: 0,
        //     y: 4,
        //     buttons: 0,
        //     wheel: 0,
        //     pan: 0,
        // };
        // push_mouse_movement(rep_up).ok().unwrap_or(0);

        // delay_ms(10);

        // debug!("mouse down");
        // let rep_down = MouseReport {
        //     x: 0,
        //     y: -4,
        //     buttons: 0,
        //     wheel: 0,
        //     pan: 0,
        // };
        // push_mouse_movement(rep_down).ok().unwrap_or(0);

        // usb_dev.poll(&mut [&mut usb_hid]);
        // info!("on!");
        // led_pin.set_high().unwrap();
        // delay.delay_ms(500);
        // info!("off!");
        // led_pin.set_low().unwrap();
        // delay.delay_ms(500);
    }
}
const KEY_A: u8 = 0x04;
const KEY_B: u8 = 0x05;
const KEY_C: u8 = 0x06;
const KEY_D: u8 = 0x07;
const KEY_E: u8 = 0x08;
const KEY_F: u8 = 0x09;
#[allow(dead_code)]
#[rustfmt::skip]
const KEY_MAP: [[u8; 2]; 3] = [
    [KEY_A, KEY_D], 
    [KEY_B, KEY_E], 
    [KEY_C, KEY_F]
];

struct KeyScanner<'a> {
    rows: [&'a mut DynPin; 3],
    cols: [&'a mut DynPin; 2],
}
impl<'a> KeyScanner<'a> {
    fn new(rows: [&'a mut DynPin; 3], cols: [&'a mut DynPin; 2]) -> Self {
        KeyScanner { rows, cols }
    }
    fn scan_matrix(&mut self) -> [[bool; 2]; 3] {
        let mut matrix = [[false; 2]; 3];
        for (y, row) in matrix.iter_mut().enumerate() {
            // info!("push!!! -- y {}", y);
            for (i, pin_row) in self.rows.iter_mut().enumerate() {
                if i == y {
                    pin_row.set_high().unwrap();
                } else {
                    pin_row.set_low().unwrap();
                }
            }
            delay_ms(5); // Wait a bit to propagete the voltage
            for (x, key) in row.iter_mut().enumerate() {
                *key = self.cols[x].is_high().unwrap();
            }
        }
        matrix
    }
    fn scan(&mut self) -> usbd_hid::descriptor::KeyboardReport {
        // debug!("scan");
        let mut key_codes = [0u8; 6];

        let matrix = self.scan_matrix();
        let mut next_keycode_index = 0;

        for (y, row) in matrix.iter().enumerate() {
            for (x, key) in row.iter().enumerate() {
                if !*key {
                    continue;
                }
                let key = KEY_MAP[y][x];
                if next_keycode_index < key_codes.len() {
                    info!("push! [{}]", key);
                    key_codes[next_keycode_index] = key;
                    next_keycode_index += 1;
                }
            }
        }
        KeyboardReport {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: key_codes,
        }
    }
}
// pub fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
//     debug!("push_mouse_movement");
//     cortex_m::interrupt::free(|_| unsafe {
//         // Now interrupts are disabled, grab the global variable and, if
//         // available, send it a HID report
//         USB_HID.as_mut().map(|hid| hid.push_input(&report))
//     })
//     .unwrap()
// }
pub fn push_key_event(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    debug!("push_key_event");
    cortex_m::interrupt::free(|_| unsafe {
        USB_HID_KEYBOARD.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    debug!("usb poll");
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    // let usb_hid = USB_HID.as_mut().unwrap();
    let usb_hid_keyboard = USB_HID_KEYBOARD.as_mut().unwrap();
    // usb_dev.poll(&mut [usb_hid, usb_hid_keyboard]);
    usb_dev.poll(&mut [usb_hid_keyboard]);
    // usb_dev.poll(&mut [usb_hid]);
}

// End of file
