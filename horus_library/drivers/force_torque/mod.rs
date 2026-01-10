//! Force/Torque Sensor drivers
//!
//! This module provides drivers for 6-axis force/torque sensors.
//!
//! # Available Drivers
//!
//! - `SimulationForceTorqueDriver` - Always available, generates synthetic data
//! - `AtiNetFtDriver` - ATI NetFT sensors (requires `netft` feature)
//! - `RobotiqSerialDriver` - Robotiq FT-300 via serial (requires `robotiq-serial` feature)

mod simulation;

#[cfg(feature = "netft")]
mod ati_netft;

#[cfg(feature = "robotiq-serial")]
mod robotiq;

// Re-exports
pub use simulation::{SimulationForceTorqueDriver, SimulationFtConfig};

#[cfg(feature = "netft")]
pub use ati_netft::{AtiNetFtConfig, AtiNetFtDriver};

#[cfg(feature = "robotiq-serial")]
pub use robotiq::{RobotiqConfig, RobotiqSerialDriver};

use crate::WrenchStamped;
use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

/// Force/torque sensor model with predefined specifications
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum FtSensorModel {
    ATI_Nano17,    // ±12 N, ±120 Nmm
    ATI_Mini40,    // ±240 N, ±6 Nm
    ATI_Mini45,    // ±290 N, ±10 Nm
    ATI_Gamma,     // ±660 N, ±60 Nm
    ATI_Delta,     // ±3300 N, ±330 Nm
    ATI_Theta,     // ±6600 N, ±660 Nm
    Robotiq_FT300, // ±300 N, ±30 Nm
    OnRobot_HexE,  // ±400 N, ±20 Nm
    Weiss_KMS40,   // ±400 N, ±15 Nm
    Optoforce_HEX, // ±200 N, ±4 Nm
    #[default]
    Generic, // User-defined ranges
}

impl FtSensorModel {
    /// Get force and torque ranges for this model
    pub fn get_ranges(&self) -> ([f64; 3], [f64; 3]) {
        match self {
            Self::ATI_Nano17 => ([12.0, 12.0, 17.0], [0.12, 0.12, 0.12]),
            Self::ATI_Mini40 => ([240.0, 240.0, 240.0], [6.0, 6.0, 6.0]),
            Self::ATI_Mini45 => ([290.0, 290.0, 580.0], [10.0, 10.0, 10.0]),
            Self::ATI_Gamma => ([660.0, 660.0, 1980.0], [60.0, 60.0, 60.0]),
            Self::ATI_Delta => ([3300.0, 3300.0, 9900.0], [330.0, 330.0, 330.0]),
            Self::ATI_Theta => ([6600.0, 6600.0, 19800.0], [660.0, 660.0, 660.0]),
            Self::Robotiq_FT300 => ([300.0, 300.0, 500.0], [30.0, 30.0, 30.0]),
            Self::OnRobot_HexE => ([400.0, 400.0, 650.0], [20.0, 20.0, 20.0]),
            Self::Weiss_KMS40 => ([400.0, 400.0, 1000.0], [15.0, 15.0, 15.0]),
            Self::Optoforce_HEX => ([200.0, 200.0, 500.0], [4.0, 4.0, 4.0]),
            Self::Generic => ([100.0, 100.0, 100.0], [10.0, 10.0, 10.0]),
        }
    }
}

/// Enum of all available force/torque driver backends
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum ForceTorqueDriverBackend {
    /// Simulation driver (always available)
    #[default]
    Simulation,
    /// ATI NetFT Ethernet sensor
    #[cfg(feature = "netft")]
    AtiNetFt,
    /// Robotiq FT-300 serial sensor
    #[cfg(feature = "robotiq-serial")]
    RobotiqSerial,
}

/// Type-erased force/torque driver for runtime backend selection
pub enum ForceTorqueDriver {
    Simulation(SimulationForceTorqueDriver),
    #[cfg(feature = "netft")]
    AtiNetFt(AtiNetFtDriver),
    #[cfg(feature = "robotiq-serial")]
    RobotiqSerial(RobotiqSerialDriver),
}

impl ForceTorqueDriver {
    /// Create a new force/torque driver with the specified backend
    pub fn new(backend: ForceTorqueDriverBackend) -> HorusResult<Self> {
        match backend {
            ForceTorqueDriverBackend::Simulation => {
                Ok(Self::Simulation(SimulationForceTorqueDriver::new()))
            }
            #[cfg(feature = "netft")]
            ForceTorqueDriverBackend::AtiNetFt => Ok(Self::AtiNetFt(AtiNetFtDriver::new()?)),
            #[cfg(feature = "robotiq-serial")]
            ForceTorqueDriverBackend::RobotiqSerial => {
                Ok(Self::RobotiqSerial(RobotiqSerialDriver::new()?))
            }
        }
    }

    /// Create a simulation driver (always available)
    pub fn simulation() -> Self {
        Self::Simulation(SimulationForceTorqueDriver::new())
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.init(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.init(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.init(),
        }
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.shutdown(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.shutdown(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.shutdown(),
        }
    }

    pub fn is_available(&self) -> bool {
        match self {
            Self::Simulation(d) => d.is_available(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.is_available(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.is_available(),
        }
    }

    pub fn status(&self) -> DriverStatus {
        match self {
            Self::Simulation(d) => d.status(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.status(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.status(),
        }
    }

    // ========================================================================
    // Sensor methods
    // ========================================================================

    pub fn read(&mut self) -> HorusResult<WrenchStamped> {
        match self {
            Self::Simulation(d) => d.read(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.read(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.read(),
        }
    }

    pub fn has_data(&self) -> bool {
        match self {
            Self::Simulation(d) => d.has_data(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.has_data(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.has_data(),
        }
    }

    pub fn sample_rate(&self) -> Option<f32> {
        match self {
            Self::Simulation(d) => d.sample_rate(),
            #[cfg(feature = "netft")]
            Self::AtiNetFt(d) => d.sample_rate(),
            #[cfg(feature = "robotiq-serial")]
            Self::RobotiqSerial(d) => d.sample_rate(),
        }
    }
}
