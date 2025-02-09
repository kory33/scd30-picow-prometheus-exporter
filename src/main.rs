//! This example test the RP Pico W on board LED.
//!
//! It does not work with the RP Pico board. See blinky.rs.

#![no_std]
#![no_main]

mod exporter_server;
mod scd30_sensor;
mod util;
mod wifi_setup;

use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_rp::peripherals::*;
use embassy_rp::{bind_interrupts, i2c, pio, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    // for debugging
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    // for I2C communication with the sensor
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    // for the PIO SPI communication with the wifi module
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
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

    static NETWORK_RESOURCES: StaticCell<
        StackResources<
            // Of these sockets, 2 will be used internally.
            // See Socket#wait_config_up for more detail.
            3,
        >,
    > = StaticCell::new();
    let (stack, mut control) = wifi_setup::init(
        &spawner,
        p.PIN_23,
        p.PIN_25,
        p.PIN_24,
        p.PIN_29,
        p.PIO0,
        p.DMA_CH0,
        NETWORK_RESOURCES.init(StackResources::new()),
        Irqs,
    )
    .await;

    loop {
        match wifi_setup::attempt_join(&mut control, env!("WIFI_NETWORK"), env!("WIFI_PASSWORD"))
            .await
        {
            Ok(_) => break,
            Err(err) => log::info!("join failed with status={}", err.status),
        }
    }
    stack.wait_config_up().await;
    log::info!("DHCP is now up!");
    log::info!("Hardware address: {:?}", stack.hardware_address());
    log::info!("IP address: {:?}", stack.config_v4());

    static LATEST_SENSOR_MEASUREMENT: StaticCell<
        embassy_sync::mutex::Mutex<
            CriticalSectionRawMutex,
            Option<scd30_interface::data::Measurement>,
        >,
    > = StaticCell::new();
    let measurement_cell = LATEST_SENSOR_MEASUREMENT.init(embassy_sync::mutex::Mutex::new(None));

    spawner
        .spawn(exporter_server::keep_serving_tcp_connections(
            stack,
            control,
            measurement_cell,
        ))
        .unwrap();
    spawner
        .spawn(scd30_sensor::keep_updating_measurement(
            scd30_sensor::setup(
                p.I2C0,  // default I2C SCS
                p.PIN_5, // default I2C SDA
                p.PIN_4, Irqs,
            ),
            measurement_cell,
        ))
        .unwrap();
}
