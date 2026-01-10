//! Simulation IMU backend
//!
//! Generates synthetic IMU data for testing and development.
//! Always available - no hardware required.

use std::time::{SystemTime, UNIX_EPOCH};

/// IMU data structure
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Orientation as quaternion [x, y, z, w]
    pub orientation: [f64; 4],
    /// Orientation covariance (row-major 3x3)
    pub orientation_covariance: [f64; 9],
    /// Angular velocity [x, y, z] in rad/s
    pub angular_velocity: [f64; 3],
    /// Angular velocity covariance (row-major 3x3)
    pub angular_velocity_covariance: [f64; 9],
    /// Linear acceleration [x, y, z] in m/s^2
    pub linear_acceleration: [f64; 3],
    /// Linear acceleration covariance (row-major 3x3)
    pub linear_acceleration_covariance: [f64; 9],
    /// Timestamp in nanoseconds since epoch
    pub timestamp: u64,
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            orientation: [0.0, 0.0, 0.0, 1.0],
            orientation_covariance: [0.0; 9],
            angular_velocity: [0.0; 3],
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: [0.0, 0.0, 9.81], // Gravity
            linear_acceleration_covariance: [0.0; 9],
            timestamp: 0,
        }
    }
}

/// Simulation IMU driver
///
/// Generates synthetic IMU data with configurable noise.
/// Useful for testing without real hardware.
pub struct SimulationImu {
    /// Target sample rate in Hz
    sample_rate: f32,
    /// Noise level (standard deviation)
    noise_level: f32,
    /// Internal state for generating data
    time_offset: f64,
    /// Last read timestamp
    last_timestamp: u64,
}

impl SimulationImu {
    /// Create a new simulation IMU
    ///
    /// # Arguments
    ///
    /// * `sample_rate` - Target sample rate in Hz
    /// * `noise_level` - Noise standard deviation (0.0 = no noise)
    pub fn new(sample_rate: f32, noise_level: f32) -> Self {
        Self {
            sample_rate,
            noise_level,
            time_offset: 0.0,
            last_timestamp: 0,
        }
    }

    /// Get the current timestamp in nanoseconds
    fn now_nanos(&self) -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    }

    /// Generate pseudo-random noise
    fn noise(&self) -> f64 {
        if self.noise_level == 0.0 {
            return 0.0;
        }

        // Simple pseudo-random using timestamp
        let t = self.now_nanos();

        (t as f64 * 0.00001).sin() * self.noise_level as f64
    }

    /// Read IMU data
    pub fn read(&mut self) -> ImuData {
        let now = self.now_nanos();
        self.last_timestamp = now;

        // Time in seconds for generating sinusoidal motion
        let t = (now as f64) / 1e9 + self.time_offset;

        // Generate gentle sinusoidal angular velocity (simulating slight rotation)
        let angular_velocity = [
            0.01 * (t * 0.5).sin() + self.noise(),
            0.01 * (t * 0.7).cos() + self.noise(),
            0.005 * (t * 0.3).sin() + self.noise(),
        ];

        // Generate acceleration (gravity + slight movement)
        let linear_acceleration = [
            0.1 * (t * 0.2).sin() + self.noise(),
            0.1 * (t * 0.3).cos() + self.noise(),
            9.81 + 0.05 * (t * 0.1).sin() + self.noise(),
        ];

        // Identity quaternion (no rotation)
        let orientation = [0.0, 0.0, 0.0, 1.0];

        // Covariance matrices (diagonal with noise level)
        let cov = self.noise_level as f64 * 0.001;
        let covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov];

        ImuData {
            orientation,
            orientation_covariance: [-1.0; 9], // No orientation estimate
            angular_velocity,
            angular_velocity_covariance: covariance,
            linear_acceleration,
            linear_acceleration_covariance: covariance,
            timestamp: now,
        }
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        true // Simulation always has data
    }

    /// Get the sample rate
    pub fn sample_rate(&self) -> f32 {
        self.sample_rate
    }

    /// Set the time offset for testing
    pub fn set_time_offset(&mut self, offset: f64) {
        self.time_offset = offset;
    }
}

// Make SimulationImu Send + Sync for use across threads
unsafe impl Send for SimulationImu {}
unsafe impl Sync for SimulationImu {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_imu_creation() {
        let imu = SimulationImu::new(100.0, 0.01);
        assert_eq!(imu.sample_rate(), 100.0);
        assert!(imu.has_data());
    }

    #[test]
    fn test_simulation_imu_read() {
        let mut imu = SimulationImu::new(100.0, 0.0);
        let data = imu.read();

        // Check gravity is approximately correct
        assert!((data.linear_acceleration[2] - 9.81).abs() < 1.0);

        // Check timestamp is set
        assert!(data.timestamp > 0);
    }

    #[test]
    fn test_simulation_imu_with_noise() {
        let mut imu = SimulationImu::new(100.0, 0.1);
        let data1 = imu.read();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let data2 = imu.read();

        // Data should vary due to noise and time
        assert_ne!(data1.timestamp, data2.timestamp);
    }
}
