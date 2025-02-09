use embassy_rp::i2c::{Async, I2c, Instance, InterruptHandler, SclPin, SdaPin};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::i2c;
use embassy_rp::{peripherals::*, Peripheral};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

pub fn setup<'d, T: Instance>(
    i2c_peripheral: impl Peripheral<P = T> + 'd,
    scl_pin: impl Peripheral<P = impl SclPin<T>> + 'd,
    sda_pin: impl Peripheral<P = impl SdaPin<T>> + 'd,
    _irq_witness: impl Binding<T::Interrupt, InterruptHandler<T>>,
) -> scd30_interface::asynch::Scd30<I2c<'d, T, Async>> {
    let i2c = i2c::I2c::new_async(i2c_peripheral, scl_pin, sda_pin, _irq_witness, {
        let mut config = i2c::Config::default();
        config.frequency = 20_000;
        config
    });

    scd30_interface::asynch::Scd30::new(i2c)
}

#[embassy_executor::task]
pub async fn keep_updating_measurement(
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
