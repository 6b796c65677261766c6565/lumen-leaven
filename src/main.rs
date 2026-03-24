#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    config::WatchdogStatus,
    delay::{Delay, MicrosDurationU64},
    gpio::{Io, Level, Output},
    prelude::*,
    timer::timg::TimerGroup,
};

/// Oven proofing setpoint (°F).
const TARGET_TEMP_F: f32 = 80.0;
/// Half-band around setpoint to reduce SSR chatter (°F).
const HYSTERESIS_F: f32 = 0.5;

#[entry]
fn main() -> ! {
    let mut config = esp_hal::Config::default();
    config.watchdog.timg0 = WatchdogStatus::Enabled(MicrosDurationU64::millis(10_000));
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut wdt = timg0.wdt;

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // GPIO1 → G3MB-202P DC+ input (active-high turns SSR / lamp on — verify your module).
    let mut ssr = Output::new(io.pins.gpio1, Level::Low);

    let delay = Delay::new();

    loop {
        wdt.feed();

        let current_temp_f = read_oven_temp_fahrenheit_stub();

        if current_temp_f < TARGET_TEMP_F - HYSTERESIS_F {
            ssr.set_high();
        } else if current_temp_f > TARGET_TEMP_F + HYSTERESIS_F {
            ssr.set_low();
        }

        delay.delay_millis(2_000);
    }
}

/// Stub until a real sensor is wired (e.g. DS18B20, I²C TMP117, thermistor + ADC).
///
/// Returns a fixed value below the setpoint so the lamp path can be verified without a sensor.
fn read_oven_temp_fahrenheit_stub() -> f32 {
    79.0
}
