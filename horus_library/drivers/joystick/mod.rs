//! Joystick/Gamepad drivers
//!
//! This module provides drivers for gamepad/joystick input.
//!
//! # Available Drivers
//!
//! - `SimulationJoystickDriver` - Always available, generates synthetic input
//! - `GilrsJoystickDriver` - Real gamepad input via gilrs (requires `gilrs` feature)

mod simulation;

#[cfg(feature = "gilrs")]
mod gilrs_driver;

// Re-exports
pub use simulation::{SimulationJoystickConfig, SimulationJoystickDriver};

#[cfg(feature = "gilrs")]
pub use gilrs_driver::{GilrsConfig, GilrsJoystickDriver};

use crate::JoystickInput;
use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

/// Button mapping profile type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ButtonMapping {
    Xbox360,
    PlayStation4,
    #[default]
    Generic,
}

/// Axis calibration data
#[derive(Debug, Clone, Copy)]
pub struct AxisCalibration {
    pub center: f32,
    pub min: f32,
    pub max: f32,
}

impl Default for AxisCalibration {
    fn default() -> Self {
        Self {
            center: 0.0,
            min: -1.0,
            max: 1.0,
        }
    }
}

/// Joystick configuration shared across backends
#[derive(Debug, Clone)]
pub struct JoystickConfig {
    /// Device ID for multi-controller setups
    pub device_id: u32,
    /// Global deadzone for all axes (0.0 to 1.0)
    pub deadzone: f32,
    /// Invert X axis
    pub invert_x: bool,
    /// Invert Y axis
    pub invert_y: bool,
    /// Invert right X axis
    pub invert_rx: bool,
    /// Invert right Y axis
    pub invert_ry: bool,
    /// Button mapping profile
    pub button_mapping: ButtonMapping,
}

impl Default for JoystickConfig {
    fn default() -> Self {
        Self {
            device_id: 0,
            deadzone: 0.1,
            invert_x: false,
            invert_y: false,
            invert_rx: false,
            invert_ry: false,
            button_mapping: ButtonMapping::Generic,
        }
    }
}

/// Enum of all available joystick driver backends
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum JoystickDriverBackend {
    /// Simulation driver (always available)
    #[default]
    Simulation,
    /// Gilrs-based gamepad driver
    #[cfg(feature = "gilrs")]
    Gilrs,
}

/// Type-erased joystick driver for runtime backend selection
#[allow(clippy::large_enum_variant)]
pub enum JoystickDriver {
    Simulation(SimulationJoystickDriver),
    #[cfg(feature = "gilrs")]
    Gilrs(GilrsJoystickDriver),
}

impl JoystickDriver {
    /// Create a new joystick driver with the specified backend
    pub fn new(backend: JoystickDriverBackend) -> HorusResult<Self> {
        match backend {
            JoystickDriverBackend::Simulation => {
                Ok(Self::Simulation(SimulationJoystickDriver::new()))
            }
            #[cfg(feature = "gilrs")]
            JoystickDriverBackend::Gilrs => Ok(Self::Gilrs(GilrsJoystickDriver::new()?)),
        }
    }

    /// Create a simulation driver (always available)
    pub fn simulation() -> Self {
        Self::Simulation(SimulationJoystickDriver::new())
    }

    /// Check if a controller is connected
    pub fn is_controller_connected(&self) -> bool {
        match self {
            Self::Simulation(d) => d.is_controller_connected(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.is_controller_connected(),
        }
    }

    /// Get battery level if supported
    pub fn get_battery_level(&self) -> Option<f32> {
        match self {
            Self::Simulation(d) => d.get_battery_level(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.get_battery_level(),
        }
    }

    /// Poll for next event (non-blocking)
    pub fn poll_event(&mut self) -> Option<JoystickInput> {
        match self {
            Self::Simulation(d) => d.poll_event(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.poll_event(),
        }
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.init(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.init(),
        }
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.shutdown(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.shutdown(),
        }
    }

    pub fn is_available(&self) -> bool {
        match self {
            Self::Simulation(d) => d.is_available(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.is_available(),
        }
    }

    pub fn status(&self) -> DriverStatus {
        match self {
            Self::Simulation(d) => d.status(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.status(),
        }
    }

    // ========================================================================
    // Sensor methods
    // ========================================================================

    pub fn read(&mut self) -> HorusResult<JoystickInput> {
        match self {
            Self::Simulation(d) => d.read(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.read(),
        }
    }

    pub fn has_data(&self) -> bool {
        match self {
            Self::Simulation(d) => d.has_data(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.has_data(),
        }
    }

    pub fn sample_rate(&self) -> Option<f32> {
        match self {
            Self::Simulation(d) => d.sample_rate(),
            #[cfg(feature = "gilrs")]
            Self::Gilrs(d) => d.sample_rate(),
        }
    }
}
