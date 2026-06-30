//! Main motor driver struct and public API.

use core::f32::consts::PI;

use embedded_hal::i2c;

use crate::error::MotorError;
use crate::registers::*;

/// Iarduino I2C motor controller driver.
///
/// Unlike the original Arduino library and many embedded-hal drivers,
/// **`Motor` does not own the I2C bus**. Every method that needs I2C
/// communication takes `&mut I2C` as its first argument.
///
/// This avoids ownership/locking issues in multi-motor setups (e.g. a
/// two-wheeled car sharing one I2C bus) and lets the caller control
/// bus lifecycle.
///
/// # Example
///
/// ```ignore
/// let i2c = /* your I2C peripheral */;
/// let mut mot_l = Motor::new(0x09);
/// let mut mot_r = Motor::new(0x0A);
///
/// mot_l.begin(&mut i2c)?;
/// mot_r.begin(&mut i2c)?;
///
/// mot_l.set_speed_rpm(&mut i2c, 1500)?;
/// mot_r.set_speed_rpm(&mut i2c, 1200)?;
/// ```
pub struct Motor {
    /// I2C address of the device.
    addr: u8,
    /// Firmware version read during [`begin`](Motor::begin).
    version: u8,
    /// Wheel radius in millimetres. Used for m/s ↔ RPM conversions.
    /// Default: 1.0 mm.
    pub radius_mm: f32,
    /// retry count for I2C operations. Default: 10.
    pub retry_count: u8,
}

impl Motor {
    // ── Construction ──────────────────────────────────────────────────────

    /// Create a new motor driver instance with a known I2C address.
    ///
    /// `addr` should be in the range 0x01–0x7E. Values >0x7F are
    /// automatically right-shifted (to handle 8-bit I2C addresses
    /// that include the R/W bit).
    pub fn new(addr: u8) -> Self {
        let addr = if addr > 0x7F { addr >> 1 } else { addr };
        Self {
            addr,
            version: 0,
            radius_mm: 1.0,
            retry_count: 10,
        }
    }

    /// Create a new motor driver instance with auto-address discovery.
    ///
    /// The address will be determined during [`begin`](Motor::begin) by
    /// scanning the I2C bus.
    pub fn new_auto() -> Self {
        Self {
            addr: 0,
            version: 0,
            radius_mm: 1.0,
            retry_count: 10,
        }
    }

    /// Return the I2C address of the device.
    pub fn address(&self) -> u8 {
        self.addr
    }

    /// Return the firmware version read during [`begin`](Motor::begin).
    pub fn version(&self) -> u8 {
        self.version
    }

    // ── Internal I2C helpers ──────────────────────────────────────────────

    /// Read bytes from a device register into `buf`.
    fn read_reg<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), MotorError> {
        let mut retrys = self.retry_count;
        loop {
            match i2c.write_read(self.addr, &[reg], buf) {
                Ok(_) => return Ok(()),
                Err(_) => {
                    if retrys == 0 {
                        return Err(MotorError::I2c);
                    }
                    retrys -= 1;
                }
            }
        }
    }

    /// Write bytes to a device register.
    fn write_reg<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        reg: u8,
        data: &[u8],
    ) -> Result<(), MotorError> {
        let n = data.len();
        let mut tx = [0u8; 32];
        tx[0] = reg;
        tx[1..][..n].copy_from_slice(data);

        let to_send = &tx[..n + 1];

        let mut retrys = self.retry_count;
        loop {
            match i2c.write(self.addr, to_send) {
                Ok(_) => return Ok(()),
                Err(_) => {
                    if retrys == 0 {
                        return Err(MotorError::I2c);
                    }
                    retrys -= 1;
                }
            }
        }
    }

    fn read_u8<I2C: i2c::I2c>(&self, i2c: &mut I2C, reg: u8) -> Result<u8, MotorError> {
        let mut buf = [0u8; 1];
        self.read_reg(i2c, reg, &mut buf)?;
        Ok(buf[0])
    }

    fn write_u8<I2C: i2c::I2c>(&self, i2c: &mut I2C, reg: u8, val: u8) -> Result<(), MotorError> {
        self.write_reg(i2c, reg, &[val])
    }

    fn read_i16_le<I2C: i2c::I2c>(&self, i2c: &mut I2C, reg: u8) -> Result<i16, MotorError> {
        let mut buf = [0u8; 2];
        self.read_reg(i2c, reg, &mut buf)?;
        Ok(i16::from_le_bytes(buf))
    }

    fn read_u24_le<I2C: i2c::I2c>(&self, i2c: &mut I2C, reg: u8) -> Result<u32, MotorError> {
        let mut buf = [0u8; 3];
        self.read_reg(i2c, reg, &mut buf)?;
        Ok(u32::from_le_bytes([buf[0], buf[1], buf[2], 0]))
    }

    fn check_address<I2C: i2c::I2c>(&self, i2c: &mut I2C, addr: u8) -> bool {
        let mut retries = self.retry_count;
        loop {
            let mut buf = [0u8; 1];
            match i2c.write_read(addr, &[REG_MODEL], &mut buf) {
                Ok(_) => return true,
                Err(_) => {
                    if retries == 0 {
                        return false;
                    }
                    retries -= 1;
                }
            }
        }
    }

    // ── Initialisation ────────────────────────────────────────────────────

    /// Initialise the motor controller.
    ///
    /// Verifies the device identity (`MODEL == 0x14`, valid `CHIP_ID`),
    /// stores the firmware version, and performs a hardware reset.
    ///
    /// If the device was created with [`new_auto`](Motor::new_auto), the
    /// I2C bus is scanned to find the motor controller.
    pub fn begin<I2C: i2c::I2c>(&mut self, i2c: &mut I2C) -> Result<(), MotorError> {
        if self.addr == 0 {
            if !self.scan_for_device(i2c) {
                return Err(MotorError::I2c);
            }
        }

        if !self.check_address(i2c, self.addr) {
            self.addr = 0;
            return Err(MotorError::I2c);
        }

        let mut id = [0u8; 4];
        self.read_reg(i2c, REG_MODEL, &mut id)?;

        if id[0] != DEF_MODEL_MOT {
            self.addr = 0;
            return Err(MotorError::InvalidParam);
        }

        let device_addr = id[2] >> 1;
        if device_addr != self.addr && id[2] != 0xFF {
            self.addr = 0;
            return Err(MotorError::InvalidParam);
        }

        let chip_id = id[3];
        if chip_id != DEF_CHIP_ID_FLASH && chip_id != DEF_CHIP_ID_METRO {
            self.addr = 0;
            return Err(MotorError::InvalidParam);
        }

        self.version = id[1];
        self.reset(i2c)?;

        Ok(())
    }

    /// Scan I2C bus for a matching motor controller.
    fn scan_for_device<I2C: i2c::I2c>(&mut self, i2c: &mut I2C) -> bool {
        for i in 1..127 {
            if !self.check_address(i2c, i) {
                continue;
            }
            self.addr = i;
            let mut id = [0u8; 4];
            if self.read_reg(i2c, REG_MODEL, &mut id).is_err() {
                self.addr = 0;
                continue;
            }
            if id[0] != DEF_MODEL_MOT {
                self.addr = 0;
                continue;
            }
            let device_addr = id[2] >> 1;
            if device_addr != i && id[2] != 0xFF {
                self.addr = 0;
                continue;
            }
            let chip_id = id[3];
            if chip_id != DEF_CHIP_ID_FLASH && chip_id != DEF_CHIP_ID_METRO {
                self.addr = 0;
                continue;
            }
            self.version = id[1];
            return true;
        }
        self.addr = 0;
        false
    }

    // ── Reset ─────────────────────────────────────────────────────────────

    /// Perform a software reset of the motor controller.
    pub fn reset<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        let bits0 = self.read_u8(i2c, REG_BITS_0)?;
        self.write_u8(i2c, REG_BITS_0, bits0 | BIT_SET_RESET)?;

        loop {
            let flg = self.read_u8(i2c, REG_FLAGS_0)?;
            if flg & FLG_RESET != 0 {
                break;
            }
        }

        Ok(())
    }

    // ── Address change ────────────────────────────────────────────────────

    /// Change the I2C address of the motor controller.
    /// Addresses 0x00 and 0x7F are forbidden.
    pub fn change_address<I2C: i2c::I2c>(
        &mut self,
        i2c: &mut I2C,
        new_addr: u8,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let new_addr = if new_addr > 0x7F {
            new_addr >> 1
        } else {
            new_addr
        };
        if new_addr == 0x00 || new_addr == 0x7F {
            return Err(MotorError::InvalidParam);
        }

        let bits0 = self.read_u8(i2c, REG_BITS_0)?;
        self.write_u8(i2c, REG_BITS_0, (bits0 & !BIT_BLOCK_ADR) | BIT_SAVE_ADR_EN)?;
        self.write_u8(i2c, REG_ADDRESS, (new_addr << 1) | 0x01)?;

        if !self.check_address(i2c, new_addr) {
            return Err(MotorError::I2c);
        }

        self.addr = new_addr;
        Ok(())
    }

    // ── I2C pull-up ───────────────────────────────────────────────────────

    /// Check if the device supports I2C pull-up control.
    pub fn has_pull_i2c<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(i2c, REG_FLAGS_0)?;
        Ok(flg & FLG_I2C_UP != 0)
    }

    /// Read the current state of the I2C pull-up resistors.
    pub fn pull_i2c<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let mut buf = [0u8; 2];
        self.read_reg(i2c, REG_FLAGS_0, &mut buf)?;
        if buf[0] & FLG_I2C_UP == 0 {
            return Err(MotorError::Unsupported);
        }
        Ok(buf[1] & BIT_SET_I2C_UP != 0)
    }

    /// Enable or disable the I2C pull-up resistors.
    pub fn set_pull_i2c<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        enable: bool,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let mut buf = [0u8; 2];
        self.read_reg(i2c, REG_FLAGS_0, &mut buf)?;
        if buf[0] & FLG_I2C_UP == 0 {
            return Err(MotorError::Unsupported);
        }
        let new_bits = if enable {
            buf[1] | BIT_SET_I2C_UP
        } else {
            buf[1] & !BIT_SET_I2C_UP
        };
        self.write_u8(i2c, REG_BITS_0, new_bits)?;
        Ok(())
    }

    // ── PWM frequency ────────────────────────────────────────────────────

    /// Set the PWM frequency (25–1000 Hz).
    pub fn set_pwm_freq<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        freq_hz: u16,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        if !(25..=1000).contains(&freq_hz) {
            return Err(MotorError::InvalidParam);
        }
        self.write_reg(i2c, REG_MOT_FREQUENCY_L, &freq_hz.to_le_bytes())
    }

    // ── Hall sensor magnets ──────────────────────────────────────────────

    /// Set the number of magnets on the rotor (1–255).
    pub fn set_magnet_count<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        count: u8,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        if count == 0 {
            return Err(MotorError::InvalidParam);
        }
        self.write_u8(i2c, REG_MOT_MAGNET, count)?;
        Ok(())
    }

    /// Read the number of magnets on the rotor.
    pub fn magnet_count<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<u8, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.read_u8(i2c, REG_MOT_MAGNET)
    }

    // ── Gear ratio ───────────────────────────────────────────────────────

    /// Set the gear ratio (0.01 .. 167_772.15).
    pub fn set_gear_ratio<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        ratio: f32,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let ratio = ratio.clamp(0.01, 167_772.15);
        let cent = (ratio * 100.0) as u32;
        let bytes = cent.to_le_bytes();
        self.write_reg(i2c, REG_MOT_REDUCER_L, &bytes[..3])
    }

    /// Read the gear ratio.
    pub fn gear_ratio<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let raw = self.read_u24_le(i2c, REG_MOT_REDUCER_L)?;
        Ok(raw as f32 / 100.0)
    }

    // ── Error threshold / flags ──────────────────────────────────────────

    /// Set the maximum allowed RPM deviation percent (0–100).
    pub fn set_error_threshold<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        pct: u8,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_u8(i2c, REG_MOT_MAX_RPM_DEV, pct.min(100))?;
        Ok(())
    }

    /// Read the current error flags.
    pub fn error_flags<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<ErrorFlags, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(i2c, REG_MOT_FLG)?;
        if flg & MOT_FLG_RPM_ERR != 0 {
            Ok(ErrorFlags::SpeedError)
        } else if flg & MOT_FLG_DRV_ERR != 0 {
            Ok(ErrorFlags::DriverError)
        } else {
            Ok(ErrorFlags::None)
        }
    }

    // ── Speed control ────────────────────────────────────────────────────

    /// Set speed in RPM (±32767).
    pub fn set_speed_rpm<I2C: i2c::I2c>(&self, i2c: &mut I2C, rpm: i16) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_reg(i2c, REG_MOT_SET_RPM_L, &rpm.to_le_bytes())
    }

    /// Set speed as raw PWM value (±4095).
    pub fn set_speed_pwm_raw<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        pwm: i16,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let pwm = pwm.clamp(-4095, 4095);
        self.write_reg(i2c, REG_MOT_SET_PWM_L, &pwm.to_le_bytes())
    }

    /// Set speed as PWM percentage (±100).
    pub fn set_speed_pwm_pct<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        pct: f32,
    ) -> Result<(), MotorError> {
        let pct = pct.clamp(-100.0, 100.0);
        let pwm = (pct * 4095.0 / 100.0) as i16;
        self.set_speed_pwm_raw(i2c, pwm)
    }

    /// Set speed in metres per second (converts via wheel radius).
    pub fn set_speed_m_per_s<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        mps: f32,
    ) -> Result<(), MotorError> {
        let mm_per_min = mps * 60_000.0;
        let circumference_mm = 2.0 * PI * self.radius_mm;
        if circumference_mm <= f32::EPSILON {
            return Err(MotorError::InvalidParam);
        }
        let rpm = (mm_per_min / circumference_mm) as i16;
        self.set_speed_rpm(i2c, rpm)
    }

    /// Set speed with a stop condition.
    pub fn set_speed_with_stop<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        speed_val: f32,
        speed_type: SpeedType,
        stop_val: f32,
        stop_type: StopType,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.set_stop_impl(i2c, stop_val, stop_type)?;
        match speed_type {
            SpeedType::Rpm => self.set_speed_rpm(i2c, speed_val as i16),
            SpeedType::Pwm => self.set_speed_pwm_pct(i2c, speed_val),
            SpeedType::MPerSec => self.set_speed_m_per_s(i2c, speed_val),
        }
    }

    /// Read the current speed.
    pub fn speed<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        speed_type: SpeedType,
    ) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        let (reg, is_pwm) = match speed_type {
            SpeedType::Rpm | SpeedType::MPerSec => (REG_MOT_GET_RPM_L, false),
            SpeedType::Pwm => (REG_MOT_SET_PWM_L, true),
        };

        let raw = self.read_i16_le(i2c, reg)?;
        let mut val = raw as f32;

        if is_pwm {
            val = val * 100.0 / 4095.0;
        } else if speed_type == SpeedType::MPerSec {
            let circumference_mm = 2.0 * PI * self.radius_mm;
            if circumference_mm > f32::EPSILON {
                val = val * circumference_mm / 60_000.0;
            }
        }

        Ok(val)
    }

    // ── Stop control ─────────────────────────────────────────────────────

    /// Stop the motor immediately.
    pub fn stop<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<(), MotorError> {
        self.set_stop_impl(i2c, 0.0, StopType::Immediate)
    }

    /// Set a stop condition.
    pub fn set_stop<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        val: f32,
        stop_type: StopType,
    ) -> Result<(), MotorError> {
        self.set_stop_impl(i2c, val, stop_type)
    }

    fn set_stop_impl<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        val: f32,
        stop_type: StopType,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        match stop_type {
            StopType::Immediate => {
                let flg = self.read_u8(i2c, REG_MOT_FLG)?;
                let neutral = if flg & MOT_FLG_NEUTRAL != 0 {
                    MOT_BIT_NEUTRAL
                } else {
                    0
                };
                self.write_u8(i2c, REG_MOT_STOP, MOT_BIT_STOP | neutral)?;
            }
            StopType::Meters => {
                let dist_mm = val * 1000.0;
                let circumference_mm = 2.0 * PI * self.radius_mm;
                if circumference_mm <= f32::EPSILON {
                    return Err(MotorError::InvalidParam);
                }
                self.set_stop_revs(i2c, dist_mm / circumference_mm)?;
            }
            StopType::Revolutions => {
                self.set_stop_revs(i2c, val)?;
            }
            StopType::Seconds => {
                let ms = (val * 1000.0) as u32;
                let ms = ms.min(16_777_215);
                self.write_reg(i2c, REG_MOT_STOP_TMR_L, &ms.to_le_bytes()[..3])?;
            }
        }

        Ok(())
    }

    fn set_stop_revs<I2C: i2c::I2c>(&self, i2c: &mut I2C, revs: f32) -> Result<(), MotorError> {
        let revs = revs.clamp(0.0, 167_772.15);
        let cent = (revs * 100.0) as u32;
        self.write_reg(i2c, REG_MOT_STOP_REV_L, &cent.to_le_bytes()[..3])
    }

    /// Read the remaining stop condition value.
    pub fn remaining_stop<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        stop_type: StopType,
    ) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        let mut buf = [0u8; 6];
        self.read_reg(i2c, REG_MOT_STOP_REV_L, &mut buf)?;

        let rev_raw = u32::from_le_bytes([buf[0], buf[1], buf[2], 0]);
        let revs = rev_raw as f32 / 100.0;
        let tmr_raw = u32::from_le_bytes([buf[3], buf[4], buf[5], 0]);

        let magnets = self.read_u8(i2c, REG_MOT_MAGNET)?;
        if magnets == 0 && stop_type != StopType::Seconds {
            return Ok(0.0);
        }

        match stop_type {
            StopType::Meters => {
                let result = if revs > 0.0 {
                    revs
                } else if tmr_raw > 0 {
                    let rpm = self.read_i16_le(i2c, REG_MOT_GET_RPM_L)?;
                    let rpm = rpm.unsigned_abs() as f32;
                    let rpm = if rpm == 0.0 { 1.0 } else { rpm };
                    rpm * tmr_raw as f32 / 60_000.0
                } else {
                    0.0
                };
                let circumference_mm = 2.0 * PI * self.radius_mm;
                Ok(result * circumference_mm / 1000.0)
            }
            StopType::Revolutions => {
                if revs > 0.0 {
                    Ok(revs)
                } else if tmr_raw > 0 {
                    let rpm = self.read_i16_le(i2c, REG_MOT_GET_RPM_L)?;
                    let rpm = rpm.unsigned_abs() as f32;
                    let rpm = if rpm == 0.0 { 1.0 } else { rpm };
                    Ok(rpm * tmr_raw as f32 / 60_000.0)
                } else {
                    Ok(0.0)
                }
            }
            StopType::Seconds => {
                if tmr_raw > 0 {
                    Ok(tmr_raw as f32 / 1000.0)
                } else if revs > 0.0 {
                    let rpm = self.read_i16_le(i2c, REG_MOT_GET_RPM_L)?;
                    let rpm = rpm.unsigned_abs() as f32;
                    let rpm = if rpm == 0.0 { 1.0 } else { rpm };
                    Ok(revs * 60.0 / rpm)
                } else {
                    Ok(0.0)
                }
            }
            StopType::Immediate => Ok(0.0),
        }
    }

    // ── Neutral stop ─────────────────────────────────────────────────────

    /// Enable or disable neutral (free-wheeling) mode when stopped.
    pub fn set_stop_neutral<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        enabled: bool,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(i2c, REG_MOT_FLG)?;
        let neutral = if enabled { MOT_BIT_NEUTRAL } else { 0 };
        let stop = if flg & MOT_FLG_STOP != 0 {
            MOT_BIT_STOP
        } else {
            0
        };
        self.write_u8(i2c, REG_MOT_STOP, neutral | stop)?;
        Ok(())
    }

    /// Read whether neutral mode is enabled.
    pub fn stop_neutral<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(i2c, REG_MOT_FLG)?;
        Ok(flg & MOT_FLG_NEUTRAL != 0)
    }

    // ── Direction ────────────────────────────────────────────────────────

    /// Set the rotation direction (`true` = clockwise at positive speed).
    pub fn set_direction<I2C: i2c::I2c>(&self, i2c: &mut I2C, ckw: bool) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(i2c, REG_BITS_2)?;
        let new_bits = if ckw {
            bits | MOT_BIT_DIR_CKW
        } else {
            bits & !MOT_BIT_DIR_CKW
        };
        self.write_u8(i2c, REG_BITS_2, new_bits)?;
        Ok(())
    }

    /// Read the direction setting.
    pub fn direction<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(i2c, REG_BITS_2)?;
        Ok(bits & MOT_BIT_DIR_CKW != 0)
    }

    // ── Inversion flags ──────────────────────────────────────────────────

    /// Set gear and motor polarity inversion flags.
    pub fn set_inversion<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        reducer_inv: bool,
        rotor_inv: bool,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(i2c, REG_BITS_2)?;
        let new_bits = bits & !(MOT_BIT_INV_RDR | MOT_BIT_INV_PIN);
        let new_bits = new_bits
            | if reducer_inv { MOT_BIT_INV_RDR } else { 0 }
            | if rotor_inv { MOT_BIT_INV_PIN } else { 0 };
        self.write_u8(i2c, REG_BITS_2, new_bits)?;
        Ok(())
    }

    /// Read the inversion flags as `(reducer_inv, rotor_inv)`.
    pub fn inversion<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<(bool, bool), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(i2c, REG_BITS_2)?;
        Ok((bits & MOT_BIT_INV_RDR != 0, bits & MOT_BIT_INV_PIN != 0))
    }

    // ── Total distance / revolutions ─────────────────────────────────────

    /// Read the total number of accumulated revolutions.
    pub fn total_revolutions<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let raw = self.read_u24_le(i2c, REG_MOT_GET_REV_L)?;
        Ok(raw as f32 / 100.0)
    }

    /// Read the total distance travelled in metres.
    pub fn total_distance_m<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let revs = self.total_revolutions(i2c)?;
        let circumference_mm = 2.0 * PI * self.radius_mm;
        Ok(revs * circumference_mm / 1000.0)
    }

    /// Reset the total revolution counter.
    pub fn reset_total<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_reg(i2c, REG_MOT_STOP_REV_L, &[0x00, 0x00, 0x00])
    }

    // ── Voltage ──────────────────────────────────────────────────────────

    /// Set the nominal motor voltage (0.0–25.5 V).
    pub fn set_nominal_voltage<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        volts: f32,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let volts = volts.clamp(0.0, 25.5);
        self.write_u8(i2c, REG_MOT_VOLTAGE, (volts * 10.0) as u8)?;
        Ok(())
    }

    /// Read the nominal motor voltage.
    pub fn nominal_voltage<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let raw = self.read_u8(i2c, REG_MOT_VOLTAGE)?;
        Ok(raw as f32 / 10.0)
    }

    // ── Nominal RPM ──────────────────────────────────────────────────────

    /// Set the nominal RPM (0–65535).
    pub fn set_nominal_rpm<I2C: i2c::I2c>(
        &self,
        i2c: &mut I2C,
        rpm: u16,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_reg(i2c, REG_MOT_NOMINAL_RPM_L, &rpm.to_le_bytes())
    }

    /// Read the nominal RPM.
    pub fn nominal_rpm<I2C: i2c::I2c>(&self, i2c: &mut I2C) -> Result<u16, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let mut buf = [0u8; 2];
        self.read_reg(i2c, REG_MOT_NOMINAL_RPM_L, &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    // ── Save manufacturer code to flash ──────────────────────────────────

    /// Save manufacturer / configuration data to flash memory.
    pub fn save_to_flash<I2C: i2c::I2c>(&self, i2c: &mut I2C, code: u64) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bytes = code.to_le_bytes();
        self.write_reg(i2c, REG_MANUFACTURER, &bytes[..5])
    }
}
