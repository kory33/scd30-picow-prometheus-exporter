use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_rp::clocks::RoscRng;
use embassy_rp::interrupt;
use embassy_rp::peripherals::*;
use embassy_rp::{gpio, pio};
use static_cell::StaticCell;

use panic_probe as _;

use rand::RngCore as _;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, gpio::Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

pub async fn init<const STACK_SIZE: usize>(
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

pub async fn attempt_join(
    control: &mut cyw43::Control<'static>,
    ssid: &str,
    password: &str,
) -> Result<(), cyw43::ControlError> {
    control
        .join(ssid, JoinOptions::new(password.as_bytes()))
        .await
}
