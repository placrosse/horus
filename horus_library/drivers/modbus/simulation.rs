//! Simulation Modbus driver

use std::collections::HashMap;

use horus_core::driver::DriverStatus;
use horus_core::error::{HorusError, HorusResult};

use super::ModbusConfig;

/// Simulation Modbus driver
///
/// Simulates Modbus device with in-memory registers.
pub struct SimulationModbusDriver {
    config: ModbusConfig,
    status: DriverStatus,
    /// Holding registers (read/write)
    holding_registers: HashMap<u16, u16>,
    /// Input registers (read-only)
    input_registers: HashMap<u16, u16>,
    /// Coils (read/write bits)
    coils: HashMap<u16, bool>,
    /// Discrete inputs (read-only bits)
    discrete_inputs: HashMap<u16, bool>,
}

impl SimulationModbusDriver {
    pub fn new(config: ModbusConfig) -> Self {
        Self {
            config,
            status: DriverStatus::Uninitialized,
            holding_registers: HashMap::new(),
            input_registers: HashMap::new(),
            coils: HashMap::new(),
            discrete_inputs: HashMap::new(),
        }
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        self.holding_registers.clear();
        self.input_registers.clear();
        self.coils.clear();
        self.discrete_inputs.clear();
        self.status = DriverStatus::Ready;
        Ok(())
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        self.status = DriverStatus::Shutdown;
        Ok(())
    }

    pub fn is_available(&self) -> bool {
        true // Simulation is always available
    }

    pub fn status(&self) -> DriverStatus {
        self.status.clone()
    }

    // ========================================================================
    // Bus methods
    // ========================================================================

    pub fn read_bytes(&mut self, addr: u16, len: usize) -> HorusResult<Vec<u8>> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        // Read holding registers and convert to bytes
        let num_regs = len.div_ceil(2);
        let mut result = Vec::with_capacity(len);

        for i in 0..num_regs as u16 {
            let value = self
                .holding_registers
                .get(&(addr + i))
                .copied()
                .unwrap_or(0);
            result.push((value >> 8) as u8);
            result.push((value & 0xFF) as u8);
        }

        result.truncate(len);
        Ok(result)
    }

    pub fn write_bytes(&mut self, addr: u16, data: &[u8]) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        // Write bytes as holding registers
        for (i, chunk) in data.chunks(2).enumerate() {
            let value = if chunk.len() == 2 {
                ((chunk[0] as u16) << 8) | (chunk[1] as u16)
            } else {
                (chunk[0] as u16) << 8
            };
            self.holding_registers.insert(addr + i as u16, value);
        }

        Ok(())
    }

    // ========================================================================
    // Modbus-specific methods
    // ========================================================================

    pub fn read_holding_registers(&mut self, addr: u16, count: u16) -> HorusResult<Vec<u16>> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        let mut result = Vec::with_capacity(count as usize);
        for i in 0..count {
            result.push(
                self.holding_registers
                    .get(&(addr + i))
                    .copied()
                    .unwrap_or(0),
            );
        }
        Ok(result)
    }

    pub fn read_input_registers(&mut self, addr: u16, count: u16) -> HorusResult<Vec<u16>> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        let mut result = Vec::with_capacity(count as usize);
        for i in 0..count {
            result.push(self.input_registers.get(&(addr + i)).copied().unwrap_or(0));
        }
        Ok(result)
    }

    pub fn write_single_register(&mut self, addr: u16, value: u16) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        self.holding_registers.insert(addr, value);
        Ok(())
    }

    pub fn write_multiple_registers(&mut self, addr: u16, values: &[u16]) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        for (i, &value) in values.iter().enumerate() {
            self.holding_registers.insert(addr + i as u16, value);
        }
        Ok(())
    }

    pub fn read_coils(&mut self, addr: u16, count: u16) -> HorusResult<Vec<bool>> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        let mut result = Vec::with_capacity(count as usize);
        for i in 0..count {
            result.push(self.coils.get(&(addr + i)).copied().unwrap_or(false));
        }
        Ok(result)
    }

    pub fn write_single_coil(&mut self, addr: u16, value: bool) -> HorusResult<()> {
        if !matches!(self.status, DriverStatus::Ready | DriverStatus::Running) {
            return Err(HorusError::driver("Driver not initialized"));
        }
        self.status = DriverStatus::Running;

        self.coils.insert(addr, value);
        Ok(())
    }

    // ========================================================================
    // Test/simulation helpers
    // ========================================================================

    /// Set a holding register value (for testing)
    pub fn set_holding_register(&mut self, addr: u16, value: u16) {
        self.holding_registers.insert(addr, value);
    }

    /// Set an input register value (for testing)
    pub fn set_input_register(&mut self, addr: u16, value: u16) {
        self.input_registers.insert(addr, value);
    }

    /// Set a coil value (for testing)
    pub fn set_coil(&mut self, addr: u16, value: bool) {
        self.coils.insert(addr, value);
    }

    /// Set a discrete input value (for testing)
    pub fn set_discrete_input(&mut self, addr: u16, value: bool) {
        self.discrete_inputs.insert(addr, value);
    }

    /// Get slave ID
    pub fn slave_id(&self) -> u8 {
        self.config.slave_id
    }
}

impl Default for SimulationModbusDriver {
    fn default() -> Self {
        Self::new(ModbusConfig::default())
    }
}
