#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
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
// use usbd_hid::descriptor::MouseReport;
use usbd_hid::descriptor::SerializedDescriptor as _;
// use usbd_hid::descriptor::KeyboardReport;
// use usbd_hid::hid_class::HIDClass;

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let usb_bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    let bus_allocator = usbd::class_prelude::UsbBusAllocator::new(usb_bus);
    // let vid_pid = usbd_hid::hid_class::UsbVidPid(0x6666, 0x1234);
    let vid_pid = usbd::device::UsbVidPid(0x3333, 0x0333);
    let mut usb_hid = usbd_hid::hid_class::HIDClass::new(
        &bus_allocator,
        usbd_hid::descriptor::KeyboardReport::desc(),
        60,
    );
    let mut usb_dev = usbd::device::UsbDeviceBuilder::new(&bus_allocator, vid_pid)
        .manufacturer("mitsumaru company")
        .product("pico-mute-key-board")
        .serial_number("333")
        .build();

    loop {
        // info!("on!");
        // led_pin.set_high().unwrap();
        // delay.delay_ms(500);
        // info!("off!");
        // led_pin.set_low().unwrap();
        // delay.delay_ms(500);

        usb_dev.poll(&mut [&mut usb_hid]);
    }
}

// End of file
