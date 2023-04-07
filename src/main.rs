//! # Edge-rs
//!
//! This application implements a Rust-based implementation of Edge-rs for the
//! Raspberry Pi Pico.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Needed for debug output symbols to be linked in binary image
use defmt_rtt as _;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_probe as _;

// Some traits we need
use core::fmt::Write;
use fugit::RateExtU32;
use rp2040_hal::clocks::Clock;

// Alias for our HAL crate
use rp2040_hal as hal;

use embedded_hal::delay::blocking::DelayUs;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

use bme280::i2c::BME280;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Until cortex_m implements the DelayUs trait needed for embedded-hal-1.0.0,
// provide a wrapper around it
pub struct DelayWrap(cortex_m::delay::Delay);

impl embedded_hal::delay::blocking::DelayUs for DelayWrap {
    type Error = core::convert::Infallible;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.0.delay_us(us);

        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.0.delay_ms(ms);
        Ok(())
    }
}

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
        clocks.peripheral_clock.freq(),
    );

    defmt::debug!("BME280 example\r\n");

    let mut delay = DelayWrap(cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().raw(),
    ));
    // initialize the BME280 using the secondary I2C address 0x77
    let mut bme280 = BME280::new_secondary(i2c);

    // initialize the sensor
    let res = bme280.init(&mut delay);
    match res {
        Ok(_) => defmt::debug!("Successfully initialized BME280 device\r\n"),
        Err(_) => defmt::debug!("Failed to initialize BME280 device\r\n"),
    }

    loop {
        // measure temperature, pressure, and humidity
        let measurements = bme280.measure(&mut delay).unwrap();

        defmt::debug!("Relative humidity: {:?}%\r", &measurements.humidity);
        defmt::debug!("Temperature: {:?} deg C\r", &measurements.temperature);
        defmt::debug!("Pressure: {:?} pascals\r\n", &measurements.pressure);

        delay.delay_ms(2000).ok().unwrap();
    }
}

// End of file
