#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use bsp::hal;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
};
use pac::interrupt;
use pac::interrupt::USBCTRL_IRQ;
use rp_pico as bsp;

use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid};

use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<UsbBus>> = None;

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
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(bus_allocator);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let vid_pid = UsbVidPid(0x16c0, 0x27da);
    let usb_dev = UsbDeviceBuilder::new(bus_ref, vid_pid)
        .manufacturer("mitsumaru company")
        .product("pico-mute-key-board")
        .serial_number("TEST")
        .device_class(0x03) // misc
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(USBCTRL_IRQ);
    };

    let sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();
    // output
    let mut row1 = pins.gpio2.into_push_pull_output();
    // let row2 = pins.gpio4.into_push_pull_output();
    // let row3 = pins.gpio3.into_push_pull_output();
    // input
    let col1 = pins.gpio19.into_pull_down_input();
    // let col2 = pins.gpio18.into_pull_down_input();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    // let key_repo_a = KeyboardReport {
    //     modifier: 0,
    //     reserved: 0,
    //     leds: 0,
    //     keycodes: [0x04 /* A */, 0, 0, 0, 0, 0],
    // };
    // let key_repo_b = KeyboardReport {
    //     modifier: 0,
    //     reserved: 0,
    //     leds: 0,
    //     keycodes: [0x05 /* B */, 0, 0, 0, 0, 0],
    // };
    // let init_key_repo = KeyboardReport {
    //     modifier: 0,
    //     reserved: 0,
    //     leds: 0,
    //     keycodes: [0x00 /* A */, 0, 0, 0, 0, 0],
    // };
    info!("Loop Start!");
    row1.set_high().unwrap();

    loop {
        if col1.is_high().unwrap() {
            led_pin.set_high().unwrap();
            // push_key_event(key_repo_a).ok().unwrap_or(0);
        } else {
            led_pin.set_low().unwrap();
            // push_key_event(init_key_repo).ok().unwrap_or(0);
        }
        // delay.delay_ms(100);

        // let rep_up = MouseReport {
        //     x: 0,
        //     y: 4,
        //     buttons: 0,
        //     wheel: 0,
        //     pan: 0,
        // };
        // push_mouse_movement(rep_up).ok().unwrap_or(0);

        // delay.delay_ms(100);

        // let rep_down = MouseReport {
        //     x: 0,
        //     y: -4,
        //     buttons: 0,
        //     wheel: 0,
        //     pan: 0,
        // };
        // push_mouse_movement(rep_down).ok().unwrap_or(0);
    }
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}
pub fn push_key_event(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    // debug!("push_key_event");
    cortex_m::interrupt::free(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
        .unwrap()
}
/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
// End of file
