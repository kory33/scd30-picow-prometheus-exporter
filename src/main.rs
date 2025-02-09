//! This example test the RP Pico W on board LED.
//!
//! It does not work with the RP Pico board. See blinky.rs.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::peripherals::{I2C0, USB};
use embassy_rp::{bind_interrupts, i2c, usb};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::task]
async fn logger_task(driver: usb::Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // prepare usb logger for debugging
    let driver = usb::Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let mut sensor = {
        let sda = p.PIN_4 /* default I2C SDA */;
        let scl = p.PIN_5 /* default I2C SCS */;
        let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, {
            let mut config = i2c::Config::default();
            config.frequency = 20_000;
            config
        });

        scd30_interface::asynch::Scd30::new(i2c)
    };

    loop {
        while sensor.is_data_ready().await != Ok(scd30_interface::data::DataStatus::Ready) {}
        let measurement = sensor.read_measurement().await.unwrap();
        log::info!("Measurement: {:?}", measurement);
        Timer::after_millis(1000).await;
    }
}
