//! HORUS IMU Driver Plugin
//!
//! This plugin provides IMU (Inertial Measurement Unit) drivers for HORUS.
//! It supports both static (compile-time) and dynamic (runtime) loading.
//!
//! # Available Backends
//!
//! - `simulation` - Always available, generates synthetic data
//! - `mpu6050` - MPU6050 6-axis IMU (requires `mpu6050` feature)
//! - `bno055` - BNO055 9-axis IMU with fusion (requires `bno055` feature)
//! - `icm20948` - ICM-20948 9-axis IMU (requires `icm20948` feature)
//!
//! # Static Usage
//!
//! ```rust,ignore
//! use horus_core::plugin::{DriverLoader, DriverLoaderConfig};
//! use horus_imu_plugin::ImuPlugin;
//!
//! let mut loader = DriverLoader::new(DriverLoaderConfig::default());
//! loader.register_static(Box::new(ImuPlugin::new()));
//! loader.initialize().unwrap();
//! ```
//!
//! # Dynamic Usage
//!
//! Place the compiled `libhorus_imu_plugin.so` in `~/.horus/drivers/` and it
//! will be discovered automatically when `driver_mode: dynamic` is set.

use std::any::Any;
use std::collections::HashMap;

use horus_core::driver::{DriverCategory, SingleDriverConfig};
use horus_core::error::{HorusError, HorusResult};
use horus_core::plugin::{BackendInfo, DriverPlugin, PluginHealth, PluginManifest, ProbeResult};

mod simulation;
pub use simulation::SimulationImu;

#[cfg(feature = "mpu6050")]
mod mpu6050_backend;
#[cfg(feature = "mpu6050")]
pub use mpu6050_backend::Mpu6050Imu;

// Plugin version (should match Cargo.toml)
const PLUGIN_VERSION: &str = env!("CARGO_PKG_VERSION");

/// IMU Driver Plugin for HORUS
///
/// Implements the [`DriverPlugin`] trait to provide IMU drivers through
/// the HORUS plugin system. Supports multiple backends including simulation
/// and hardware drivers.
pub struct ImuPlugin {
    /// Number of active driver instances (for future use)
    #[allow(dead_code)]
    active_instances: u32,
}

impl ImuPlugin {
    /// Create a new IMU plugin instance
    pub fn new() -> Self {
        Self {
            active_instances: 0,
        }
    }

    /// Get list of supported backends based on compiled features
    #[allow(unused_mut)] // May be mutable depending on features
    fn available_backends() -> Vec<BackendInfo> {
        let mut backends = vec![BackendInfo {
            id: "simulation".into(),
            name: "Simulation IMU".into(),
            description: "Generates synthetic IMU data for testing and development".into(),
            hardware_ids: vec![],
        }];

        #[cfg(feature = "mpu6050")]
        backends.push(BackendInfo {
            id: "mpu6050".into(),
            name: "MPU6050".into(),
            description: "InvenSense MPU6050 6-axis IMU (accelerometer + gyroscope)".into(),
            hardware_ids: vec!["i2c:0x68".into(), "i2c:0x69".into()],
        });

        #[cfg(feature = "bno055")]
        backends.push(BackendInfo {
            id: "bno055".into(),
            name: "BNO055".into(),
            description: "Bosch BNO055 9-axis IMU with sensor fusion".into(),
            hardware_ids: vec!["i2c:0x28".into(), "i2c:0x29".into()],
        });

        #[cfg(feature = "icm20948")]
        backends.push(BackendInfo {
            id: "icm20948".into(),
            name: "ICM-20948".into(),
            description: "TDK ICM-20948 9-axis IMU".into(),
            hardware_ids: vec!["i2c:0x68".into(), "i2c:0x69".into()],
        });

        backends
    }
}

impl Default for ImuPlugin {
    fn default() -> Self {
        Self::new()
    }
}

impl DriverPlugin for ImuPlugin {
    fn manifest(&self) -> PluginManifest {
        PluginManifest {
            id: "horus-imu".into(),
            name: "HORUS IMU Plugin".into(),
            version: PLUGIN_VERSION.into(),
            category: DriverCategory::Sensor,
            description:
                "IMU (Inertial Measurement Unit) drivers for accelerometer and gyroscope sensors"
                    .into(),
            author: Some("HORUS Team".into()),
            license: Some("MIT OR Apache-2.0".into()),
            repository: Some("https://github.com/softmata/horus".into()),
            backends: Self::available_backends(),
            system_deps: vec![],
            horus_version: ">=0.1.0".into(),
            platforms: vec!["linux".into(), "macos".into()],
            features: vec![],
            extra: HashMap::new(),
        }
    }

    #[allow(clippy::vec_init_then_push)]
    fn probe(&self, backend_id: &str) -> Vec<ProbeResult> {
        match backend_id {
            "" | "all" => {
                // Probe all backends
                let mut results = Vec::new();

                // Simulation is always available
                results.push(ProbeResult::detected("simulation", "virtual"));

                #[cfg(feature = "mpu6050")]
                {
                    if let Some(result) = probe_i2c_device("mpu6050", &[0x68, 0x69]) {
                        results.push(result);
                    }
                }

                #[cfg(feature = "bno055")]
                {
                    if let Some(result) = probe_i2c_device("bno055", &[0x28, 0x29]) {
                        results.push(result);
                    }
                }

                #[cfg(feature = "icm20948")]
                {
                    if let Some(result) = probe_i2c_device("icm20948", &[0x68, 0x69]) {
                        results.push(result);
                    }
                }

                results
            }
            "simulation" => {
                vec![ProbeResult::detected("simulation", "virtual")]
            }
            #[cfg(feature = "mpu6050")]
            "mpu6050" => {
                if let Some(result) = probe_i2c_device("mpu6050", &[0x68, 0x69]) {
                    vec![result]
                } else {
                    vec![ProbeResult::not_detected("mpu6050")]
                }
            }
            #[cfg(feature = "bno055")]
            "bno055" => {
                if let Some(result) = probe_i2c_device("bno055", &[0x28, 0x29]) {
                    vec![result]
                } else {
                    vec![ProbeResult::not_detected("bno055")]
                }
            }
            #[cfg(feature = "icm20948")]
            "icm20948" => {
                if let Some(result) = probe_i2c_device("icm20948", &[0x68, 0x69]) {
                    vec![result]
                } else {
                    vec![ProbeResult::not_detected("icm20948")]
                }
            }
            _ => {
                vec![ProbeResult::not_detected(backend_id)
                    .with_message(format!("Unknown backend: {}", backend_id))]
            }
        }
    }

    fn create(
        &self,
        backend_id: &str,
        config: &SingleDriverConfig,
    ) -> HorusResult<Box<dyn Any + Send + Sync>> {
        match backend_id {
            "simulation" => {
                // Use fps field for sample rate, or get from options
                let sample_rate = config
                    .fps
                    .or_else(|| config.get_option_f64("sample_rate").map(|v| v as f32))
                    .unwrap_or(100.0);

                // Noise level is a custom option
                let noise_level = config
                    .get_option_f64("noise_level")
                    .map(|v| v as f32)
                    .unwrap_or(0.01);

                let imu = SimulationImu::new(sample_rate, noise_level);
                Ok(Box::new(imu))
            }
            #[cfg(feature = "mpu6050")]
            "mpu6050" => {
                // Build I2C bus path from bus number
                let i2c_bus = config
                    .i2c_bus
                    .map(|b| format!("/dev/i2c-{}", b))
                    .or_else(|| config.get_option("i2c_bus"))
                    .unwrap_or_else(|| "/dev/i2c-1".to_string());

                let address = config
                    .i2c_address
                    .or_else(|| config.get_option_i64("address").map(|v| v as u8))
                    .unwrap_or(0x68);

                let imu = Mpu6050Imu::new(&i2c_bus, address)?;
                Ok(Box::new(imu))
            }
            _ => Err(HorusError::driver(format!(
                "Unknown IMU backend: {}. Available: {:?}",
                backend_id,
                Self::available_backends()
                    .iter()
                    .map(|b| b.id.as_str())
                    .collect::<Vec<_>>()
            ))),
        }
    }

    fn health(&self) -> PluginHealth {
        PluginHealth::healthy("horus-imu")
    }

    fn on_load(&self) -> HorusResult<()> {
        // Plugin loaded successfully
        Ok(())
    }

    fn on_unload(&self) {
        // Cleanup if needed
    }
}

/// Probe for an I2C device at the given addresses
#[cfg(any(feature = "mpu6050", feature = "bno055", feature = "icm20948"))]
fn probe_i2c_device(backend_id: &str, addresses: &[u8]) -> Option<ProbeResult> {
    // Check common I2C buses
    let buses = ["/dev/i2c-1", "/dev/i2c-0", "/dev/i2c-2"];

    for bus in &buses {
        if !std::path::Path::new(bus).exists() {
            continue;
        }

        // For each address, try to detect the device
        for addr in addresses {
            // Simple check - try to access the bus
            // In a real implementation, we'd actually probe the device
            if std::path::Path::new(bus).exists() {
                return Some(
                    ProbeResult::with_confidence(
                        backend_id,
                        0.5, // Medium confidence - bus exists but device not verified
                        Some(format!("{}@0x{:02x}", bus, addr)),
                    )
                    .with_metadata("i2c_bus", bus.to_string())
                    .with_metadata("address", format!("0x{:02x}", addr))
                    .with_message("I2C bus found - device not verified"),
                );
            }
        }
    }

    None
}

// =============================================================================
// Dynamic plugin entry point
// =============================================================================

/// Entry point for dynamic plugin loading
///
/// This function is called by the HORUS plugin loader when loading
/// this plugin dynamically. It returns a boxed instance of the plugin.
///
/// # Safety
///
/// This is an unsafe FFI function that must follow C calling conventions.
/// The returned Box must be valid and properly initialized.
#[no_mangle]
#[allow(improper_ctypes_definitions)]
pub extern "C" fn horus_driver_entry() -> Box<dyn DriverPlugin> {
    Box::new(ImuPlugin::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_manifest() {
        let plugin = ImuPlugin::new();
        let manifest = plugin.manifest();

        assert_eq!(manifest.id, "horus-imu");
        assert_eq!(manifest.category, DriverCategory::Sensor);
        assert!(!manifest.backends.is_empty());

        // Simulation backend should always be present
        assert!(manifest.backends.iter().any(|b| b.id == "simulation"));
    }

    #[test]
    fn test_probe_simulation() {
        let plugin = ImuPlugin::new();
        let results = plugin.probe("simulation");

        assert_eq!(results.len(), 1);
        assert!(results[0].detected);
        assert_eq!(results[0].backend_id, "simulation");
    }

    #[test]
    fn test_probe_all() {
        let plugin = ImuPlugin::new();
        let results = plugin.probe("");

        // Should at least have simulation
        assert!(!results.is_empty());
        assert!(results.iter().any(|r| r.backend_id == "simulation"));
    }

    #[test]
    fn test_create_simulation() {
        let plugin = ImuPlugin::new();
        let config = SingleDriverConfig::default();

        let driver = plugin.create("simulation", &config);
        assert!(driver.is_ok());

        // Verify it's a SimulationImu
        let driver = driver.unwrap();
        assert!(driver.downcast_ref::<SimulationImu>().is_some());
    }

    #[test]
    fn test_create_unknown_backend() {
        let plugin = ImuPlugin::new();
        let config = SingleDriverConfig::default();

        let result = plugin.create("nonexistent", &config);
        assert!(result.is_err());
    }

    #[test]
    fn test_health() {
        let plugin = ImuPlugin::new();
        let health = plugin.health();

        assert!(health.healthy);
        assert_eq!(health.plugin_id, "horus-imu");
    }
}
