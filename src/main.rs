//! This example test the RP Pico W on board LED.
//!
//! It does not work with the RP Pico board. See blinky.rs.

#![no_std]
#![no_main]

use core2::io::Cursor;
use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::StackResources;
use embassy_rp::clocks::RoscRng;
use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::*;
use embassy_rp::{bind_interrupts, gpio, i2c, interrupt, pio, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use core2::io::Write as _;
use embedded_io_async::Write as _;
use rand::RngCore as _;

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

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, gpio::Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

fn clone_measurement(
    measurement: &scd30_interface::data::Measurement,
) -> scd30_interface::data::Measurement {
    // tbh Copy should have been derived for Measurement...
    scd30_interface::data::Measurement {
        co2_concentration: measurement.co2_concentration,
        temperature: measurement.temperature,
        humidity: measurement.humidity,
    }
}

async fn write_to_socket_or_log_error(socket: &mut TcpSocket<'_>, buf: &[u8]) {
    match socket.write_all(buf).await {
        Ok(()) => {}
        Err(e) => {
            log::warn!("write error: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn keep_serving_tcp_connections(
    stack: embassy_net::Stack<'static>,
    mut control: cyw43::Control<'static>,
    measurement_cell: &'static embassy_sync::mutex::Mutex<
        CriticalSectionRawMutex,
        Option<scd30_interface::data::Measurement>,
    >,
) -> ! {
    loop {
        let mut rx_buffer = [0; 128];
        let mut tx_buffer = [0; 1024];
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(3)));

        control.gpio_set(0, false).await;
        log::info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }

        log::info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        let measurement = {
            let guard = measurement_cell.lock().await;
            guard.as_ref().map(clone_measurement)
        };

        if let Some(measurement) = measurement {
            let mut header_buf = [0 as u8; 64];
            let mut body_buf = [0 as u8; 256];

            let response_body = {
                let mut cur = Cursor::new(&mut body_buf[..]);
                let _ = match write!(
                    &mut cur,
                    "cos_concentration_ppm {:.2}\r\ntemperature_deg_celsius {:.2}\r\nrelative_humidity_percent {:.2}\r\n",
                    measurement.co2_concentration, measurement.temperature, measurement.humidity
                ) {
                    Ok(()) => {}
                    Err(e) => {
                        log::warn!("write error: {:?}", e);
                        continue;
                    }
                };
                let position = cur.position();
                &body_buf[..position as usize]
            };

            let response_header = {
                let mut cur = Cursor::new(&mut header_buf[..]);
                match write!(
                    &mut cur,
                    "HTTP/1.1 200 OK\r\nContent-Length: {}\r\n\r\n",
                    response_body.len()
                ) {
                    Ok(_) => {}
                    Err(e) => {
                        log::warn!("write error: {:?}", e);
                        continue;
                    }
                };
                let position = cur.position();
                &header_buf[..position as usize]
            };

            write_to_socket_or_log_error(&mut socket, response_header).await;
            write_to_socket_or_log_error(&mut socket, response_body).await;
            match socket.flush().await {
                Ok(()) => {}
                Err(e) => {
                    log::warn!("flush error: {:?}", e);
                }
            };
            socket.close();
        } else {
            write_to_socket_or_log_error(&mut socket, b"Waiting for sensor data...\n").await
        }
    }
}

#[embassy_executor::task]
async fn keep_updating_measurement(
    mut sensor: scd30_interface::asynch::Scd30<I2c<'static, I2C0, Async>>,
    measurement_cell: &'static embassy_sync::mutex::Mutex<
        CriticalSectionRawMutex,
        Option<scd30_interface::data::Measurement>,
    >,
) {
    loop {
        while sensor.is_data_ready().await != Ok(scd30_interface::data::DataStatus::Ready) {
            Timer::after(Duration::from_millis(1000)).await;
        }

        match sensor.read_measurement().await {
            Ok(measurement) => {
                let mut guard = measurement_cell.lock().await;
                *guard = Some(measurement);
            }
            Err(e) => {
                log::warn!("error reading measurement: {:?}", e);
            }
        }
    }
}

async fn setup_wifi_stack<const STACK_SIZE: usize>(
    spawner: &Spawner,
    wl_on_gpio_pin: PIN_23,
    wl_cs_gpio_pin: PIN_25,
    wl_d_gpio_pin_for_piospi: PIN_24,
    wl_clk_gpio_pin_for_piospi: PIN_29,
    pio_for_piospi: PIO0,
    dma_for_piospi: DMA_CH0,
    network_stack_resources: &'static mut StackResources<STACK_SIZE>,
    irqs: impl interrupt::typelevel::Binding<
        <PIO0 as pio::Instance>::Interrupt,
        pio::InterruptHandler<PIO0>,
    >,
) -> (embassy_net::Stack<'static>, cyw43::Control<'static>) {
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let mut rng = RoscRng;

    let pwr = gpio::Output::new(wl_on_gpio_pin, gpio::Level::Low);
    let cs = gpio::Output::new(wl_cs_gpio_pin, gpio::Level::High);
    let mut pio = pio::Pio::new(pio_for_piospi, irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        wl_d_gpio_pin_for_piospi,
        wl_clk_gpio_pin_for_piospi,
        dma_for_piospi,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = embassy_net::Config::dhcpv4(Default::default());

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    let (stack, runner) = embassy_net::new(net_device, config, network_stack_resources, seed);

    spawner.spawn(net_task(runner)).unwrap();

    (stack, control)
}

async fn attempt_wifi_join(
    control: &mut cyw43::Control<'static>,
    ssid: &str,
    password: &str,
) -> Result<(), cyw43::ControlError> {
    control
        .join(ssid, JoinOptions::new(password.as_bytes()))
        .await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // prepare usb logger for debugging
    let driver = usb::Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let sensor = {
        let sda = p.PIN_4 /* default I2C SDA */;
        let scl = p.PIN_5 /* default I2C SCS */;
        let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, {
            let mut config = i2c::Config::default();
            config.frequency = 20_000;
            config
        });

        scd30_interface::asynch::Scd30::new(i2c)
    };

    static NETWORK_RESOURCES: StaticCell<
        StackResources<
            // Of these sockets, 2 will be used internally.
            // See Socket#wait_config_up for more detail.
            3,
        >,
    > = StaticCell::new();
    let (stack, mut control) = setup_wifi_stack(
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
        match attempt_wifi_join(&mut control, env!("WIFI_NETWORK"), env!("WIFI_PASSWORD")).await {
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
        .spawn(keep_serving_tcp_connections(
            stack,
            control,
            measurement_cell,
        ))
        .unwrap();
    spawner
        .spawn(keep_updating_measurement(sensor, measurement_cell))
        .unwrap();
}
