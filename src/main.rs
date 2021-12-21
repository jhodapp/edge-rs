//! # Edge-rs
//!
//! This application implements a Rust-based implementation of Edge-rs for the
//! Raspberry Pi Pico.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Some traits we need
use embedded_time::rate::Extensions;
use core::fmt::Write;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

use bme280::BME280;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::sio::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut uart = hal::uart::UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
    .enable(
        hal::uart::common_configs::_115200_8_N_1,
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
    let _tx_pin = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
    // UART RX (characters reveived by RP2040) on pin 2 (GPIO1)
    let _rx_pin = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio18.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio19.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::i2c::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    uart.write_full_blocking(b"BME280 example\r\n");

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    // initialize the BME280 using the secondary I2C address 0x77
    let mut bme280 = BME280::new_secondary(i2c);

    // initialize the sensor
    let res = bme280.init(&mut delay);
    match res {
        Ok(_) => uart.write_full_blocking(b"Successfully initialized BME280 device\r\n"),
        Err(_) => uart.write_full_blocking(b"Failed to initialize BME280 device\r\n"),
    }

    loop {
        // measure temperature, pressure, and humidity
        let measurements = bme280.measure(&mut delay).unwrap();
        
        writeln!(uart, "Relative humidity: {:?}%\r", &measurements.humidity).ok().unwrap();
        writeln!(uart, "Temperature: {:?} deg C\r", &measurements.temperature).ok().unwrap();
        writeln!(uart, "Pressure: {:?} pascals\r\n", &measurements.pressure).ok().unwrap();

        delay.delay_ms(2000);
    }
}

// End of file