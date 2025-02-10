use core2::io::Cursor;
use embassy_net::tcp::TcpSocket;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_time::Duration;
use {defmt_rtt as _, panic_probe as _};

use core2::io::Write as _;
use embedded_io_async::Write as _;

async fn write_to_socket_or_log_error(socket: &mut TcpSocket<'_>, buf: &[u8]) {
    match socket.write_all(buf).await {
        Ok(()) => {}
        Err(e) => {
            log::warn!("write error: {:?}", e);
        }
    }
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
        let mut header_buf = [0_u8; 64];
        let mut body_buf = [0_u8; 256];

        let response_body = {
            let mut cur = Cursor::new(&mut body_buf[..]);
            match write!(
              &mut cur,
              "cos_concentration_ppm {:.2}\r\ntemperature_deg_celsius {:.2}\r\nrelative_humidity_percent {:.2}\r\n",
              measurement.co2_concentration, measurement.temperature, measurement.humidity
          ) {
              Ok(()) => {}
              Err(e) => {
                  log::warn!("write error: {:?}", e);
                  return;
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
                    return;
                }
            };
            let position = cur.position();
            &header_buf[..position as usize]
        };

        write_to_socket_or_log_error(&mut socket, response_header).await;
        write_to_socket_or_log_error(&mut socket, response_body).await;

        response_code = b"200";
    } else {
        write_to_socket_or_log_error(&mut socket, b"HTTP/1.1 503 Service Unavailable\r\n\r\n")
            .await;

        response_code = b"503";
    }

    match socket.flush().await {
        Ok(()) => {}
        Err(e) => {
            log::warn!(
                "exporter server: flush error on {:?}: {:?}",
                response_code,
                e
            );
        }
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
