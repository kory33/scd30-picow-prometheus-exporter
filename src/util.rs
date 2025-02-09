pub fn clone_measurement(
    measurement: &scd30_interface::data::Measurement,
) -> scd30_interface::data::Measurement {
    // tbh Copy should have been derived for Measurement...
    scd30_interface::data::Measurement {
        co2_concentration: measurement.co2_concentration,
        temperature: measurement.temperature,
        humidity: measurement.humidity,
    }
}
