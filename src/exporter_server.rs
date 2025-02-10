use core2::io::Cursor;
use embassy_net::tcp::TcpSocket;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_time::Duration;
use {defmt_rtt as _, panic_probe as _};

use core2::io::Write as _;
use embedded_io_async::Write as _;

async fn write_to_socket_or_log_error(socket: &mut TcpSocket<'_>, buf: &[u8]) {
    if let Err(e) = socket.write_all(buf).await {
        log::warn!("write error: {:?}", e);
    }
}

fn write_to_initial_segment_and_reborrow<'a>(
    buf: &'a mut [u8],
    fmt: core::fmt::Arguments<'_>,
) -> &'a mut [u8] {
    let mut cur = Cursor::new(&mut *buf);
    if let Err(e) = cur.write_fmt(fmt) {
        log::warn!("server: buffer write error: {:?}", e);
    }
    let position = cur.position();
    &mut buf[..position as usize]
}

async fn serve_one_http_request(
    stack: embassy_net::Stack<'_>,
    control: &mut cyw43::Control<'_>,
    measurement_cell: &embassy_sync::mutex::Mutex<
        impl RawMutex,
        Option<scd30_interface::data::Measurement>,
    >,
) {
    let mut rx_buffer = [0; 128];
    let mut tx_buffer = [0; 1024];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(3)));

    // TODO: what does gpio_set(0, _) do?
    control.gpio_set(0, false).await;
    log::info!("Listening on TCP:1234...");
    if let Err(e) = socket.accept(1234).await {
        log::warn!("accept error: {:?}", e);
        return;
    }
    log::info!("Received connection from {:?}", socket.remote_endpoint());
    control.gpio_set(0, true).await;

    let measurement = {
        let guard = measurement_cell.lock().await;
        guard.as_ref().map(crate::util::clone_measurement)
    };

    let response_code;
    if let Some(measurement) = measurement {
        let mut body_buf = [0_u8; 256];
        let response_body = write_to_initial_segment_and_reborrow(&mut body_buf,
            format_args!(
                "cos_concentration_ppm {:.2}\r\ntemperature_deg_celsius {:.2}\r\nrelative_humidity_percent {:.2}\r\n",
                measurement.co2_concentration, measurement.temperature, measurement.humidity
            )
        );

        let mut header_buf = [0_u8; 64];
        let response_header = write_to_initial_segment_and_reborrow(
            &mut header_buf,
            format_args!(
                "HTTP/1.1 200 OK\r\nContent-Length: {}\r\n\r\n",
                response_body.len()
            ),
        );

        write_to_socket_or_log_error(&mut socket, response_header).await;
        write_to_socket_or_log_error(&mut socket, response_body).await;
        response_code = 200;
    } else {
        write_to_socket_or_log_error(&mut socket, b"HTTP/1.1 503 Service Unavailable\r\n\r\n")
            .await;
        response_code = 503;
    }

    if let Error(e) = socket.flush().await {
        log::warn!(
            "exporter server: flush error on {:?}: {:?}",
            response_code,
            e
        );
    };
    socket.close();
}

#[embassy_executor::task]
pub async fn keep_serving_tcp_connections(
    stack: embassy_net::Stack<'static>,
    mut control: cyw43::Control<'static>,
    measurement_cell: &'static embassy_sync::mutex::Mutex<
        CriticalSectionRawMutex,
        Option<scd30_interface::data::Measurement>,
    >,
) -> ! {
    loop {
        serve_one_http_request(stack, &mut control, measurement_cell).await;
    }
}
