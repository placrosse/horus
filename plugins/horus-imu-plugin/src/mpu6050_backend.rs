//! MPU6050 IMU backend
//!
//! Hardware driver for the InvenSense MPU6050 6-axis IMU.
//! Requires the `mpu6050` feature to be enabled.

use std::time::{SystemTime, UNIX_EPOCH};

use horus_core::error::{HorusError, HorusResult};
use linux_embedded_hal::I2cdev;
use mpu6050::Mpu6050 as Mpu6050Device;

use crate::simulation::ImuData;

/// MPU6050 6-axis IMU driver
///
/// Provides accelerometer and gyroscope data from the MPU6050 sensor.
/// Does not provide orientation (quaternion) - only raw sensor data.
pub struct Mpu6050Imu {
    /// The MPU6050 device handle
    device: Mpu6050Device<I2cdev>,
    /// I2C bus path for reference
    #[allow(dead_code)]
    i2c_bus: String,
    /// I2C address
    #[allow(dead_code)]
    address: u8,
}

impl Mpu6050Imu {
    /// Create a new MPU6050 IMU driver
    ///
    /// # Arguments
    ///
    /// * `i2c_bus` - Path to the I2C bus (e.g., "/dev/i2c-1")
    /// * `address` - I2C address of the device (typically 0x68 or 0x69)
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C bus cannot be opened or the device
    /// cannot be initialized.
    pub fn new(i2c_bus: &str, address: u8) -> HorusResult<Self> {
        // Open the I2C bus
        let i2c = I2cdev::new(i2c_bus).map_err(|e| {
            HorusError::driver(format!("Failed to open I2C bus {}: {}", i2c_bus, e))
        })?;

        // Create the MPU6050 device
        let mut device = Mpu6050Device::new_with_addr(i2c, address);

        // Initialize the device
        device
            .init(&mut linux_embedded_hal::Delay)
            .map_err(|e| HorusError::driver(format!("Failed to initialize MPU6050: {:?}", e)))?;

        Ok(Self {
            device,
            i2c_bus: i2c_bus.to_string(),
            address,
        })
    }

    /// Get the current timestamp in nanoseconds
    fn now_nanos(&self) -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    }

    /// Read IMU data from the sensor
    pub fn read(&mut self) -> HorusResult<ImuData> {
        let now = self.now_nanos();

        // Read accelerometer data (returns [x, y, z] in g)
        let accel = self
            .device
            .get_acc()
            .map_err(|e| HorusError::driver(format!("Failed to read accelerometer: {:?}", e)))?;

        // Read gyroscope data (returns [x, y, z] in deg/s)
        let gyro = self
            .device
            .get_gyro()
            .map_err(|e| HorusError::driver(format!("Failed to read gyroscope: {:?}", e)))?;

        // Convert accelerometer from g to m/s^2
        let linear_acceleration = [
            accel.x as f64 * 9.81,
            accel.y as f64 * 9.81,
            accel.z as f64 * 9.81,
        ];

        // Convert gyroscope from deg/s to rad/s
        let angular_velocity = [
            gyro.x as f64 * std::f64::consts::PI / 180.0,
            gyro.y as f64 * std::f64::consts::PI / 180.0,
            gyro.z as f64 * std::f64::consts::PI / 180.0,
        ];

        // MPU6050 doesn't provide orientation estimation
        // Use identity quaternion and mark covariance as unknown (-1)
        Ok(ImuData {
            orientation: [0.0, 0.0, 0.0, 1.0],
            orientation_covariance: [-1.0; 9], // Unknown - no orientation estimate
            angular_velocity,
            angular_velocity_covariance: [0.0; 9], // TODO: Calibrate noise characteristics
            linear_acceleration,
            linear_acceleration_covariance: [0.0; 9], // TODO: Calibrate noise characteristics
            timestamp: now,
        })
    }

    /// Check if the device is responsive
    pub fn is_connected(&mut self) -> bool {
        // Try to read the WHO_AM_I register (should return 0x68)
        self.device.get_acc().is_ok()
    }
}

#[cfg(test)]
mod tests {
    // Hardware tests would require actual MPU6050 hardware
    // These tests are marked as ignored by default

    #[test]
    #[ignore]
    fn test_mpu6050_connection() {
        // This test requires actual hardware
        // Run with: cargo test --features mpu6050 -- --ignored
        use super::*;

        let result = Mpu6050Imu::new("/dev/i2c-1", 0x68);
        assert!(result.is_ok(), "Failed to connect to MPU6050");
    }

    #[test]
    #[ignore]
    fn test_mpu6050_read() {
        use super::*;

        let mut imu = Mpu6050Imu::new("/dev/i2c-1", 0x68).expect("Failed to connect");
        let data = imu.read().expect("Failed to read IMU data");

        // Check that timestamp is set
        assert!(data.timestamp > 0);

        // Check that we're reading gravity (approximately 9.81 m/s^2 on z-axis when flat)
        let accel_magnitude = (data.linear_acceleration[0].powi(2)
            + data.linear_acceleration[1].powi(2)
            + data.linear_acceleration[2].powi(2))
        .sqrt();
        assert!(
            (accel_magnitude - 9.81).abs() < 1.0,
            "Unexpected acceleration magnitude: {}",
            accel_magnitude
        );
    }
}
