#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use bsp::hal;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::DynPin,
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
};
use rp_pico as bsp;

use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};

use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External high-speed crystal on the pico board is 12Mhz
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

    // Set up the USB driver
    let usb_bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    let bus_allocator = UsbBusAllocator::new(usb_bus);

    let mut usb_hid = HIDClass::new(&bus_allocator, KeyboardReport::desc(), 10);

    // Create a USB device with a fake VID and PID
    let vid_pid = UsbVidPid(0x16c0, 0x27da);
    let mut usb_dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid)
        .manufacturer("mitsumaru company")
        .product("pico-mute-key-board")
        .serial_number("TEST")
        .device_class(0x03) // misc
        .build();

    let sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();
    // output
    let row1 = pins.gpio2.into_pull_up_input();
    let row2 = pins.gpio4.into_pull_up_input();
    let row3 = pins.gpio3.into_pull_up_input();
    // input
    let col1 = pins.gpio19.into_push_pull_output();
    let col2 = pins.gpio18.into_push_pull_output();

    let rows: [&mut DynPin; 3] = [&mut row1.into(), &mut row2.into(), &mut row3.into()];
    let cols: [&mut DynPin; 2] = [&mut col1.into(), &mut col2.into()];

    let mut key_scanner = KeyScanner::new(rows, cols);
    let mut last_keycodes = [0u8; 6];

    // let core = pac::CorePeripherals::take().unwrap();
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let init_key_repo = KeyboardReport {
        modifier: 0,
        reserved: 0,
        leds: 0,
        keycodes: [0x00, 0, 0, 0, 0, 0],
    };
    info!("Loop Start!");
    // let mut count = 0;

    // col1.set_low().unwrap();
    // col2.set_high().unwrap();
    // asm::delay(1000);

    loop {
        usb_dev.poll(&mut [&mut usb_hid]);
        // count += 1;
        let key_repo = key_scanner.scan();
        if last_keycodes != key_repo.keycodes {
            // debug!(
            //     "change keycodes!!! {},{},{},{},{},{}",
            //     key_repo.keycodes[0],
            //     key_repo.keycodes[1],
            //     key_repo.keycodes[2],
            //     key_repo.keycodes[3],
            //     key_repo.keycodes[4],
            //     key_repo.keycodes[5]
            // );
            led_pin.set_high().unwrap();
            usb_hid.push_input(&key_repo).ok();
            last_keycodes = key_repo.keycodes;
        } else {
            led_pin.set_low().unwrap();
            // usb_hid.push_input(&init_key_repo).ok();
        }
        // drop received data
        // usb_hid.pull_raw_output(&mut [0; 64]).ok();
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
const KEY_MAP: [[u8; 3]; 2] = [
    [KEY_A, KEY_B, KEY_C], 
    [KEY_D, KEY_E, KEY_F], 
];
struct KeyScanner<'a> {
    rows: [&'a mut DynPin; 3],
    cols: [&'a mut DynPin; 2],
}
impl<'a> KeyScanner<'a> {
    fn new(rows: [&'a mut DynPin; 3], cols: [&'a mut DynPin; 2]) -> Self {
        KeyScanner { rows, cols }
    }
    fn scan_matrix(&mut self) -> [[bool; 3]; 2] {
        let mut matrix = [[false; 3]; 2];
        for (y, row) in matrix.iter_mut().enumerate() {
            // info!("push!!! -- y {}", y);
            for (i, pin_col) in self.cols.iter_mut().enumerate() {
                if i == y {
                    // info!("low!!! -- y {},{}", y, i);
                    pin_col.set_low().unwrap();
                } else {
                    // info!("high!!! -- y {},{}", y, i);
                    pin_col.set_high().unwrap();
                }
            }
            asm::delay(1000); // Wait a bit to propagate the voltage
            for (x, key) in row.iter_mut().enumerate() {
                // info!("y{},x{}", y, x);
                *key = self.rows[x].is_low().unwrap();
                // if self.rows[x].is_low().unwrap() {
                // info!("low!{}", x);
                // }
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
                    // info!("push! [{}][{}]", key, next_keycode_index);
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
