//! This application uses a RP2040 (Pico) and an ultrasonic transducer to demonstrate
//! a level detector with hysteresis (Schmitt trigger).
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;

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
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // Configure GPIO25 as LED output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    // Configure GPIO3 as trigger output
    let mut trigger_pin = pins.gpio8.into_push_pull_output();
    // Configure GPIO2 as echo input
    let echo_pin = pins.gpio9.into_pull_down_input();

    let mut t1: u64;
    let mut t2: u64;
    let mut distance: f64;
    let mut first_time: bool = true;

    loop {
        if first_time {
            delay.delay_ms(1000);   // Initial delay to let transducer settle.
            first_time = false;
        }
        // Send ultrasonic pulse
        trigger_pin.set_low().unwrap();
        delay.delay_us(2);
        trigger_pin.set_high().unwrap();
        delay.delay_us(5);
        trigger_pin.set_low().unwrap();

        // Mark the start time (µs).
        t1 = timer.get_counter();

        // Wait until the echo pulse is received.
        while echo_pin.is_low().unwrap() {
        }
        while echo_pin.is_high().unwrap() {
        }

        // Mark the echo return time (µs).
        t2 = timer.get_counter();

        // Calculate the pulse distance using 343m/s as the speed of sound.
        // (Divide by 2 as the echo travels twice the distance.)
        distance = (t2 - t1) as f64 * 0.000343 / 2.0;
        
        // Depending on the distance do something with the LED.
        if distance > 0.8 {
            led_pin.set_high().unwrap();
        } else if distance < 0.5 {
            led_pin.set_low().unwrap();
        }
    }
}

// End of file
