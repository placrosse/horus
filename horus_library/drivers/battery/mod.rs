//! Battery Monitor drivers
//!
//! This module provides drivers for battery monitoring.
//!
//! # Available Drivers
//!
//! - `SimulationBatteryDriver` - Always available, generates synthetic battery data
//! - `I2cBatteryDriver` - I2C power monitors (INA219/INA226, requires `i2c-hardware` feature)

mod simulation;

#[cfg(feature = "i2c-hardware")]
mod i2c;

// Re-exports
pub use simulation::{SimulationBatteryConfig, SimulationBatteryDriver};

#[cfg(feature = "i2c-hardware")]
pub use i2c::{I2cBatteryConfig, I2cBatteryDriver};

use crate::BatteryState;
use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

/// Battery chemistry type
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum BatteryChemistry {
    #[default]
    LiPo, // Lithium Polymer
    LiFePO4,  // Lithium Iron Phosphate
    LiIon,    // Lithium Ion
    NiMH,     // Nickel Metal Hydride
    LeadAcid, // Lead Acid
    Custom,
}

/// Common battery configuration shared across drivers
#[derive(Debug, Clone)]
pub struct BatteryConfig {
    /// Number of cells in series
    pub cell_count: u8,
    /// Nominal voltage per cell
    pub nominal_voltage_per_cell: f32,
    /// Battery capacity in mAh
    pub capacity_mah: f32,
    /// Battery chemistry type
    pub chemistry: BatteryChemistry,
    /// Full voltage threshold
    pub full_voltage: f32,
    /// Low voltage warning threshold
    pub low_voltage: f32,
    /// Critical voltage threshold
    pub critical_voltage: f32,
    /// Enable individual cell monitoring
    pub enable_cell_monitoring: bool,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            cell_count: 3,                 // 3S default
            nominal_voltage_per_cell: 3.7, // LiPo
            capacity_mah: 5000.0,
            chemistry: BatteryChemistry::LiPo,
            full_voltage: 12.6,    // 4.2V × 3
            low_voltage: 10.5,     // 3.5V × 3
            critical_voltage: 9.9, // 3.3V × 3
            enable_cell_monitoring: false,
        }
    }
}

impl BatteryConfig {
    /// Create a new battery configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Set number of cells and recalculate thresholds
    pub fn with_cells(mut self, count: u8) -> Self {
        self.cell_count = count.clamp(1, 16);
        self.update_thresholds();
        self
    }

    /// Set battery chemistry and recalculate thresholds
    pub fn with_chemistry(mut self, chemistry: BatteryChemistry) -> Self {
        self.chemistry = chemistry;
        self.nominal_voltage_per_cell = match chemistry {
            BatteryChemistry::LiPo => 3.7,
            BatteryChemistry::LiFePO4 => 3.2,
            BatteryChemistry::LiIon => 3.6,
            BatteryChemistry::NiMH => 1.2,
            BatteryChemistry::LeadAcid => 2.0,
            BatteryChemistry::Custom => self.nominal_voltage_per_cell,
        };
        self.update_thresholds();
        self
    }

    /// Set battery capacity in mAh
    pub fn with_capacity(mut self, capacity_mah: f32) -> Self {
        self.capacity_mah = capacity_mah;
        self
    }

    /// Enable cell monitoring
    pub fn with_cell_monitoring(mut self, enable: bool) -> Self {
        self.enable_cell_monitoring = enable;
        self
    }

    /// Update voltage thresholds based on chemistry
    fn update_thresholds(&mut self) {
        let cells = self.cell_count as f32;
        match self.chemistry {
            BatteryChemistry::LiPo => {
                self.full_voltage = 4.2 * cells;
                self.low_voltage = 3.5 * cells;
                self.critical_voltage = 3.3 * cells;
            }
            BatteryChemistry::LiFePO4 => {
                self.full_voltage = 3.65 * cells;
                self.low_voltage = 3.0 * cells;
                self.critical_voltage = 2.5 * cells;
            }
            BatteryChemistry::LiIon => {
                self.full_voltage = 4.2 * cells;
                self.low_voltage = 3.4 * cells;
                self.critical_voltage = 3.0 * cells;
            }
            BatteryChemistry::NiMH => {
                self.full_voltage = 1.4 * cells;
                self.low_voltage = 1.0 * cells;
                self.critical_voltage = 0.9 * cells;
            }
            BatteryChemistry::LeadAcid => {
                self.full_voltage = 2.15 * cells;
                self.low_voltage = 1.85 * cells;
                self.critical_voltage = 1.75 * cells;
            }
            BatteryChemistry::Custom => {}
        }
    }

    /// Configure for 3S LiPo
    pub fn lipo_3s(capacity_mah: f32) -> Self {
        Self::new()
            .with_chemistry(BatteryChemistry::LiPo)
            .with_cells(3)
            .with_capacity(capacity_mah)
    }

    /// Configure for 4S LiPo
    pub fn lipo_4s(capacity_mah: f32) -> Self {
        Self::new()
            .with_chemistry(BatteryChemistry::LiPo)
            .with_cells(4)
            .with_capacity(capacity_mah)
    }

    /// Configure for 6S LiPo
    pub fn lipo_6s(capacity_mah: f32) -> Self {
        Self::new()
            .with_chemistry(BatteryChemistry::LiPo)
            .with_cells(6)
            .with_capacity(capacity_mah)
    }
}

/// Enum of all available battery driver backends
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum BatteryDriverBackend {
    /// Simulation driver (always available)
    #[default]
    Simulation,
    /// I2C power monitor (INA219/INA226)
    #[cfg(feature = "i2c-hardware")]
    I2c,
}

/// Type-erased battery driver for runtime backend selection
pub enum BatteryDriver {
    Simulation(SimulationBatteryDriver),
    #[cfg(feature = "i2c-hardware")]
    I2c(I2cBatteryDriver),
}

impl BatteryDriver {
    /// Create a new battery driver with the specified backend
    pub fn new(backend: BatteryDriverBackend) -> HorusResult<Self> {
        match backend {
            BatteryDriverBackend::Simulation => {
                Ok(Self::Simulation(SimulationBatteryDriver::new()))
            }
            #[cfg(feature = "i2c-hardware")]
            BatteryDriverBackend::I2c => Ok(Self::I2c(I2cBatteryDriver::new()?)),
        }
    }

    /// Create a simulation driver (always available)
    pub fn simulation() -> Self {
        Self::Simulation(SimulationBatteryDriver::new())
    }

    /// Create a simulation driver with custom battery configuration
    pub fn simulation_with_config(config: BatteryConfig) -> Self {
        Self::Simulation(SimulationBatteryDriver::with_battery_config(config))
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.init(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.init(),
        }
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.shutdown(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.shutdown(),
        }
    }

    pub fn is_available(&self) -> bool {
        match self {
            Self::Simulation(d) => d.is_available(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.is_available(),
        }
    }

    pub fn status(&self) -> DriverStatus {
        match self {
            Self::Simulation(d) => d.status(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.status(),
        }
    }

    // ========================================================================
    // Sensor methods
    // ========================================================================

    pub fn read(&mut self) -> HorusResult<BatteryState> {
        match self {
            Self::Simulation(d) => d.read(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.read(),
        }
    }

    pub fn has_data(&self) -> bool {
        match self {
            Self::Simulation(d) => d.has_data(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.has_data(),
        }
    }

    pub fn sample_rate(&self) -> Option<f32> {
        match self {
            Self::Simulation(d) => d.sample_rate(),
            #[cfg(feature = "i2c-hardware")]
            Self::I2c(d) => d.sample_rate(),
        }
    }
}
