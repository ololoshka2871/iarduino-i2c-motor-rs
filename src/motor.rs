//! Main motor driver struct and public API.

use core::f32::consts::PI;

use embedded_hal::i2c;

use crate::error::MotorError;
use crate::registers::*;

/// Iarduino I2C motor controller driver.
///
/// Generic over the I2C bus type. Synchronous methods require
/// [`embedded_hal::i2c::I2c`]. Async methods (behind `feature = "async"`)
/// are accessed via the [`MotorAsyncExt`] trait.
///
/// # Example (synchronous)
///
/// ```ignore
/// let i2c = /* your I2C peripheral implementing embedded_hal::i2c::I2c */;
/// let mut motor = Motor::new(i2c, 0x09);
/// motor.begin()?;
/// motor.set_speed_rpm(1500)?;
/// ```
pub struct Motor<I2C> {
    i2c: I2C,
    /// I2C address of the device.
    addr: u8,
    /// Firmware version read during [`begin`](Motor::begin).
    version: u8,
    /// Wheel radius in millimetres. Used for m/s ↔ RPM conversions.
    /// Default: 1.0 mm.
    pub radius_mm: f32,
}

// ──────────────────────────────────────────────────────────────────────────────
// Internal I2C helpers (sync)
// ──────────────────────────────────────────────────────────────────────────────

impl<I2C: i2c::I2c> Motor<I2C> {
    /// Read bytes from a device register into `buf`.
    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), MotorError> {
        self.i2c
            .write_read(self.addr, &[reg], buf)
            .map_err(|_| MotorError::I2c)
    }

    /// Write bytes to a device register.
    fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), MotorError> {
        let n = data.len();
        let mut buf = [0u8; 32];
        buf[0] = reg;
        buf[1..][..n].copy_from_slice(data);
        self.i2c
            .write(self.addr, &buf[..n + 1])
            .map_err(|_| MotorError::I2c)
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, MotorError> {
        let mut buf = [0u8; 1];
        self.read_reg(reg, &mut buf)?;
        Ok(buf[0])
    }

    fn write_u8(&mut self, reg: u8, val: u8) -> Result<u8, MotorError> {
        self.write_reg(reg, &[val])?;
        Ok(val)
    }

    fn read_i16_le(&mut self, reg: u8) -> Result<i16, MotorError> {
        let mut buf = [0u8; 2];
        self.read_reg(reg, &mut buf)?;
        Ok(i16::from_le_bytes(buf))
    }

    fn read_u24_le(&mut self, reg: u8) -> Result<u32, MotorError> {
        let mut buf = [0u8; 3];
        self.read_reg(reg, &mut buf)?;
        Ok(u32::from_le_bytes([buf[0], buf[1], buf[2], 0]))
    }

    fn check_address(&mut self, addr: u8) -> bool {
        self.i2c.write(addr, &[]).is_ok()
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Public synchronous API
// ──────────────────────────────────────────────────────────────────────────────

impl<I2C: i2c::I2c> Motor<I2C> {
    /// Create a new motor driver instance with a known I2C address.
    ///
    /// `addr` should be in the range 0x01–0x7E. Values >0x7F are
    /// automatically right-shifted (to handle 8-bit I2C addresses
    /// that include the R/W bit).
    pub fn new(i2c: I2C, addr: u8) -> Self {
        let addr = if addr > 0x7F { addr >> 1 } else { addr };
        Self {
            i2c,
            addr,
            version: 0,
            radius_mm: 1.0,
        }
    }

    /// Create a new motor driver instance with auto-address discovery.
    ///
    /// The address will be determined during [`begin`](Motor::begin) by
    /// scanning the I2C bus.
    pub fn new_auto(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: 0,
            version: 0,
            radius_mm: 1.0,
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

    // ── Initialisation ────────────────────────────────────────────────────

    /// Initialise the motor controller.
    ///
    /// Verifies the device identity (`MODEL == 0x14`, valid `CHIP_ID`),
    /// stores the firmware version, and performs a hardware reset.
    ///
    /// If the device was created with [`new_auto`](Motor::new_auto), the
    /// I2C bus is scanned to find the motor controller.
    pub fn begin(&mut self) -> Result<(), MotorError> {
        if self.addr == 0 {
            if !self.scan_for_device() {
                return Err(MotorError::I2c);
            }
        }

        if !self.check_address(self.addr) {
            self.addr = 0;
            return Err(MotorError::I2c);
        }

        let mut id = [0u8; 4];
        self.read_reg(REG_MODEL, &mut id)?;

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
        self.reset()?;

        Ok(())
    }

    /// Scan I2C bus for a matching motor controller.
    fn scan_for_device(&mut self) -> bool {
        for i in 1..127 {
            if !self.check_address(i) {
                continue;
            }
            self.addr = i;
            let mut id = [0u8; 4];
            if self.read_reg(REG_MODEL, &mut id).is_err() {
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
    pub fn reset(&mut self) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        let bits0 = self.read_u8(REG_BITS_0)?;
        self.write_u8(REG_BITS_0, bits0 | BIT_SET_RESET)?;

        loop {
            let flg = self.read_u8(REG_FLAGS_0)?;
            if flg & FLG_RESET != 0 {
                break;
            }
        }

        Ok(())
    }

    // ── Address change ────────────────────────────────────────────────────

    /// Change the I2C address of the motor controller.
    /// Addresses 0x00 and 0x7F are forbidden.
    pub fn change_address(&mut self, new_addr: u8) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let new_addr = if new_addr > 0x7F { new_addr >> 1 } else { new_addr };
        if new_addr == 0x00 || new_addr == 0x7F {
            return Err(MotorError::InvalidParam);
        }

        let bits0 = self.read_u8(REG_BITS_0)?;
        self.write_u8(REG_BITS_0, (bits0 & !BIT_BLOCK_ADR) | BIT_SAVE_ADR_EN)?;
        self.write_u8(REG_ADDRESS, (new_addr << 1) | 0x01)?;

        if !self.check_address(new_addr) {
            return Err(MotorError::I2c);
        }

        self.addr = new_addr;
        Ok(())
    }

    // ── I2C pull-up ───────────────────────────────────────────────────────

    /// Check if the device supports I2C pull-up control.
    pub fn has_pull_i2c(&mut self) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(REG_FLAGS_0)?;
        Ok(flg & FLG_I2C_UP != 0)
    }

    /// Read the current state of the I2C pull-up resistors.
    pub fn pull_i2c(&mut self) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let mut buf = [0u8; 2];
        self.read_reg(REG_FLAGS_0, &mut buf)?;
        if buf[0] & FLG_I2C_UP == 0 {
            return Err(MotorError::Unsupported);
        }
        Ok(buf[1] & BIT_SET_I2C_UP != 0)
    }

    /// Enable or disable the I2C pull-up resistors.
    pub fn set_pull_i2c(&mut self, enable: bool) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let mut buf = [0u8; 2];
        self.read_reg(REG_FLAGS_0, &mut buf)?;
        if buf[0] & FLG_I2C_UP == 0 {
            return Err(MotorError::Unsupported);
        }
        let new_bits = if enable { buf[1] | BIT_SET_I2C_UP } else { buf[1] & !BIT_SET_I2C_UP };
        self.write_u8(REG_BITS_0, new_bits)?;
        Ok(())
    }

    // ── PWM frequency ────────────────────────────────────────────────────

    /// Set the PWM frequency (25–1000 Hz).
    pub fn set_pwm_freq(&mut self, freq_hz: u16) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        if !(25..=1000).contains(&freq_hz) {
            return Err(MotorError::InvalidParam);
        }
        self.write_reg(REG_MOT_FREQUENCY_L, &freq_hz.to_le_bytes())
    }

    // ── Hall sensor magnets ──────────────────────────────────────────────

    /// Set the number of magnets on the rotor (1–255).
    pub fn set_magnet_count(&mut self, count: u8) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        if count == 0 {
            return Err(MotorError::InvalidParam);
        }
        self.write_u8(REG_MOT_MAGNET, count)?;
        Ok(())
    }

    /// Read the number of magnets on the rotor.
    pub fn magnet_count(&mut self) -> Result<u8, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.read_u8(REG_MOT_MAGNET)
    }

    // ── Gear ratio ───────────────────────────────────────────────────────

    /// Set the gear ratio (0.01 .. 167_772.15).
    pub fn set_gear_ratio(&mut self, ratio: f32) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let ratio = ratio.clamp(0.01, 167_772.15);
        let cent = (ratio * 100.0) as u32;
        let bytes = cent.to_le_bytes();
        self.write_reg(REG_MOT_REDUCER_L, &bytes[..3])
    }

    /// Read the gear ratio.
    pub fn gear_ratio(&mut self) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let raw = self.read_u24_le(REG_MOT_REDUCER_L)?;
        Ok(raw as f32 / 100.0)
    }

    // ── Error threshold / flags ──────────────────────────────────────────

    /// Set the maximum allowed RPM deviation percent (0–100).
    pub fn set_error_threshold(&mut self, pct: u8) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_u8(REG_MOT_MAX_RPM_DEV, pct.min(100))?;
        Ok(())
    }

    /// Read the current error flags.
    pub fn error_flags(&mut self) -> Result<ErrorFlags, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(REG_MOT_FLG)?;
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
    pub fn set_speed_rpm(&mut self, rpm: i16) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_reg(REG_MOT_SET_RPM_L, &rpm.to_le_bytes())
    }

    /// Set speed as raw PWM value (±4095).
    pub fn set_speed_pwm_raw(&mut self, pwm: i16) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let pwm = pwm.clamp(-4095, 4095);
        self.write_reg(REG_MOT_SET_PWM_L, &pwm.to_le_bytes())
    }

    /// Set speed as PWM percentage (±100).
    pub fn set_speed_pwm_pct(&mut self, pct: f32) -> Result<(), MotorError> {
        let pct = pct.clamp(-100.0, 100.0);
        let pwm = (pct * 4095.0 / 100.0) as i16;
        self.set_speed_pwm_raw(pwm)
    }

    /// Set speed in metres per second (converts via wheel radius).
    pub fn set_speed_m_per_s(&mut self, mps: f32) -> Result<(), MotorError> {
        let mm_per_min = mps * 60_000.0;
        let circumference_mm = 2.0 * PI * self.radius_mm;
        if circumference_mm <= f32::EPSILON {
            return Err(MotorError::InvalidParam);
        }
        let rpm = (mm_per_min / circumference_mm) as i16;
        self.set_speed_rpm(rpm)
    }

    /// Set speed with a stop condition.
    pub fn set_speed_with_stop(
        &mut self,
        speed_val: f32,
        speed_type: SpeedType,
        stop_val: f32,
        stop_type: StopType,
    ) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.set_stop_impl(stop_val, stop_type)?;
        match speed_type {
            SpeedType::Rpm => self.set_speed_rpm(speed_val as i16),
            SpeedType::Pwm => self.set_speed_pwm_pct(speed_val),
            SpeedType::MPerSec => self.set_speed_m_per_s(speed_val),
        }
    }

    /// Read the current speed.
    pub fn speed(&mut self, speed_type: SpeedType) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        let (reg, is_pwm) = match speed_type {
            SpeedType::Rpm | SpeedType::MPerSec => (REG_MOT_GET_RPM_L, false),
            SpeedType::Pwm => (REG_MOT_SET_PWM_L, true),
        };

        let raw = self.read_i16_le(reg)?;
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
    pub fn stop(&mut self) -> Result<(), MotorError> {
        self.set_stop_impl(0.0, StopType::Immediate)
    }

    /// Set a stop condition.
    pub fn set_stop(&mut self, val: f32, stop_type: StopType) -> Result<(), MotorError> {
        self.set_stop_impl(val, stop_type)
    }

    fn set_stop_impl(&mut self, val: f32, stop_type: StopType) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        match stop_type {
            StopType::Immediate => {
                let flg = self.read_u8(REG_MOT_FLG)?;
                let neutral = if flg & MOT_FLG_NEUTRAL != 0 { MOT_BIT_NEUTRAL } else { 0 };
                self.write_u8(REG_MOT_STOP, MOT_BIT_STOP | neutral)?;
            }
            StopType::Meters => {
                let dist_mm = val * 1000.0;
                let circumference_mm = 2.0 * PI * self.radius_mm;
                if circumference_mm <= f32::EPSILON {
                    return Err(MotorError::InvalidParam);
                }
                self.set_stop_revs(dist_mm / circumference_mm)?;
            }
            StopType::Revolutions => {
                self.set_stop_revs(val)?;
            }
            StopType::Seconds => {
                let ms = (val * 1000.0) as u32;
                let ms = ms.min(16_777_215);
                self.write_reg(REG_MOT_STOP_TMR_L, &ms.to_le_bytes()[..3])?;
            }
        }

        Ok(())
    }

    fn set_stop_revs(&mut self, revs: f32) -> Result<(), MotorError> {
        let revs = revs.clamp(0.0, 167_772.15);
        let cent = (revs * 100.0) as u32;
        self.write_reg(REG_MOT_STOP_REV_L, &cent.to_le_bytes()[..3])
    }

    /// Read the remaining stop condition value.
    pub fn remaining_stop(&mut self, stop_type: StopType) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }

        let mut buf = [0u8; 6];
        self.read_reg(REG_MOT_STOP_REV_L, &mut buf)?;

        let rev_raw = u32::from_le_bytes([buf[0], buf[1], buf[2], 0]);
        let revs = rev_raw as f32 / 100.0;
        let tmr_raw = u32::from_le_bytes([buf[3], buf[4], buf[5], 0]);

        let magnets = self.read_u8(REG_MOT_MAGNET)?;
        if magnets == 0 && stop_type != StopType::Seconds {
            return Ok(0.0);
        }

        match stop_type {
            StopType::Meters => {
                let result = if revs > 0.0 {
                    revs
                } else if tmr_raw > 0 {
                    let rpm = self.read_i16_le(REG_MOT_GET_RPM_L)?;
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
                    let rpm = self.read_i16_le(REG_MOT_GET_RPM_L)?;
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
                    let rpm = self.read_i16_le(REG_MOT_GET_RPM_L)?;
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
    pub fn set_stop_neutral(&mut self, enabled: bool) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(REG_MOT_FLG)?;
        let neutral = if enabled { MOT_BIT_NEUTRAL } else { 0 };
        let stop = if flg & MOT_FLG_STOP != 0 { MOT_BIT_STOP } else { 0 };
        self.write_u8(REG_MOT_STOP, neutral | stop)?;
        Ok(())
    }

    /// Read whether neutral mode is enabled.
    pub fn stop_neutral(&mut self) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let flg = self.read_u8(REG_MOT_FLG)?;
        Ok(flg & MOT_FLG_NEUTRAL != 0)
    }

    // ── Direction ────────────────────────────────────────────────────────

    /// Set the rotation direction (`true` = clockwise at positive speed).
    pub fn set_direction(&mut self, ckw: bool) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(REG_BITS_2)?;
        let new_bits = if ckw { bits | MOT_BIT_DIR_CKW } else { bits & !MOT_BIT_DIR_CKW };
        self.write_u8(REG_BITS_2, new_bits)?;
        Ok(())
    }

    /// Read the direction setting.
    pub fn direction(&mut self) -> Result<bool, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(REG_BITS_2)?;
        Ok(bits & MOT_BIT_DIR_CKW != 0)
    }

    // ── Inversion flags ──────────────────────────────────────────────────

    /// Set gear and motor polarity inversion flags.
    pub fn set_inversion(&mut self, reducer_inv: bool, rotor_inv: bool) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(REG_BITS_2)?;
        let new_bits = bits & !(MOT_BIT_INV_RDR | MOT_BIT_INV_PIN);
        let new_bits = new_bits
            | if reducer_inv { MOT_BIT_INV_RDR } else { 0 }
            | if rotor_inv { MOT_BIT_INV_PIN } else { 0 };
        self.write_u8(REG_BITS_2, new_bits)?;
        Ok(())
    }

    /// Read the inversion flags as `(reducer_inv, rotor_inv)`.
    pub fn inversion(&mut self) -> Result<(bool, bool), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bits = self.read_u8(REG_BITS_2)?;
        Ok((
            bits & MOT_BIT_INV_RDR != 0,
            bits & MOT_BIT_INV_PIN != 0,
        ))
    }

    // ── Total distance / revolutions ─────────────────────────────────────

    /// Read the total number of accumulated revolutions.
    pub fn total_revolutions(&mut self) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let raw = self.read_u24_le(REG_MOT_GET_REV_L)?;
        Ok(raw as f32 / 100.0)
    }

    /// Read the total distance travelled in metres.
    pub fn total_distance_m(&mut self) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let revs = self.total_revolutions()?;
        let circumference_mm = 2.0 * PI * self.radius_mm;
        Ok(revs * circumference_mm / 1000.0)
    }

    /// Reset the total revolution counter.
    pub fn reset_total(&mut self) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_reg(REG_MOT_STOP_REV_L, &[0x00, 0x00, 0x00])
    }

    // ── Voltage ──────────────────────────────────────────────────────────

    /// Set the nominal motor voltage (0.0–25.5 V).
    pub fn set_nominal_voltage(&mut self, volts: f32) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let volts = volts.clamp(0.0, 25.5);
        self.write_u8(REG_MOT_VOLTAGE, (volts * 10.0) as u8)?;
        Ok(())
    }

    /// Read the nominal motor voltage.
    pub fn nominal_voltage(&mut self) -> Result<f32, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let raw = self.read_u8(REG_MOT_VOLTAGE)?;
        Ok(raw as f32 / 10.0)
    }

    // ── Nominal RPM ──────────────────────────────────────────────────────

    /// Set the nominal RPM (0–65535).
    pub fn set_nominal_rpm(&mut self, rpm: u16) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        self.write_reg(REG_MOT_NOMINAL_RPM_L, &rpm.to_le_bytes())
    }

    /// Read the nominal RPM.
    pub fn nominal_rpm(&mut self) -> Result<u16, MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let mut buf = [0u8; 2];
        self.read_reg(REG_MOT_NOMINAL_RPM_L, &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    // ── Save manufacturer code to flash ──────────────────────────────────

    /// Save manufacturer / configuration data to flash memory.
    pub fn save_to_flash(&mut self, code: u64) -> Result<(), MotorError> {
        if self.addr == 0 {
            return Err(MotorError::NotInitialized);
        }
        let bytes = code.to_le_bytes();
        self.write_reg(REG_MANUFACTURER, &bytes[..5])
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Async API (feature = "async")
//
// Async methods are exposed via the MotorAsyncExt trait. Users import the trait
// and call the same method names with `.await`.
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "async")]
mod asyn {
    use core::f32::consts::PI;

    use crate::error::MotorError;
    use crate::registers::*;
    use crate::Motor;

    /// Async I2C transport trait for the motor driver.
    ///
    /// Implement this for your async I2C bus to enable async motor operations.
    #[allow(async_fn_in_trait)]
    pub trait AsyncI2c {
        /// Error type, must implement [`embedded_io::Error`].
        type Error: embedded_io::Error;

        /// Combined write-read: send `reg` then read `buf` with repeated start.
        async fn write_read(
            &mut self,
            addr: u8,
            reg: u8,
            buf: &mut [u8],
        ) -> Result<(), Self::Error>;

        /// Write `[reg, data[0], data[1], ...]` to the device.
        async fn write(&mut self, addr: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error>;
    }

    /// Extension trait adding async methods to [`Motor`].
    ///
    /// Import this trait to use `motor.method().await` on any [`Motor`] whose
    /// I2C type implements [`AsyncI2c`].
    #[allow(async_fn_in_trait)]
    pub trait MotorAsyncExt {
        async fn begin(&mut self) -> Result<(), MotorError>;
        async fn reset(&mut self) -> Result<(), MotorError>;
        async fn change_address(&mut self, new_addr: u8) -> Result<(), MotorError>;
        async fn has_pull_i2c(&mut self) -> Result<bool, MotorError>;
        async fn pull_i2c(&mut self) -> Result<bool, MotorError>;
        async fn set_pull_i2c(&mut self, enable: bool) -> Result<(), MotorError>;
        async fn set_pwm_freq(&mut self, freq_hz: u16) -> Result<(), MotorError>;
        async fn set_magnet_count(&mut self, count: u8) -> Result<(), MotorError>;
        async fn magnet_count(&mut self) -> Result<u8, MotorError>;
        async fn set_gear_ratio(&mut self, ratio: f32) -> Result<(), MotorError>;
        async fn gear_ratio(&mut self) -> Result<f32, MotorError>;
        async fn set_error_threshold(&mut self, pct: u8) -> Result<(), MotorError>;
        async fn error_flags(&mut self) -> Result<ErrorFlags, MotorError>;
        async fn set_speed_rpm(&mut self, rpm: i16) -> Result<(), MotorError>;
        async fn set_speed_pwm_raw(&mut self, pwm: i16) -> Result<(), MotorError>;
        async fn set_speed_pwm_pct(&mut self, pct: f32) -> Result<(), MotorError>;
        async fn set_speed_m_per_s(&mut self, mps: f32) -> Result<(), MotorError>;
        async fn set_speed_with_stop(
            &mut self,
            speed_val: f32,
            speed_type: SpeedType,
            stop_val: f32,
            stop_type: StopType,
        ) -> Result<(), MotorError>;
        async fn speed(&mut self, speed_type: SpeedType) -> Result<f32, MotorError>;
        async fn stop(&mut self) -> Result<(), MotorError>;
        async fn set_stop(&mut self, val: f32, stop_type: StopType) -> Result<(), MotorError>;
        async fn remaining_stop(&mut self, stop_type: StopType) -> Result<f32, MotorError>;
        async fn set_stop_neutral(&mut self, enabled: bool) -> Result<(), MotorError>;
        async fn stop_neutral(&mut self) -> Result<bool, MotorError>;
        async fn set_direction(&mut self, ckw: bool) -> Result<(), MotorError>;
        async fn direction(&mut self) -> Result<bool, MotorError>;
        async fn set_inversion(
            &mut self,
            reducer_inv: bool,
            rotor_inv: bool,
        ) -> Result<(), MotorError>;
        async fn inversion(&mut self) -> Result<(bool, bool), MotorError>;
        async fn total_revolutions(&mut self) -> Result<f32, MotorError>;
        async fn total_distance_m(&mut self) -> Result<f32, MotorError>;
        async fn reset_total(&mut self) -> Result<(), MotorError>;
        async fn set_nominal_voltage(&mut self, volts: f32) -> Result<(), MotorError>;
        async fn nominal_voltage(&mut self) -> Result<f32, MotorError>;
        async fn set_nominal_rpm(&mut self, rpm: u16) -> Result<(), MotorError>;
        async fn nominal_rpm(&mut self) -> Result<u16, MotorError>;
        async fn save_to_flash(&mut self, code: u64) -> Result<(), MotorError>;
    }

    // ── Internal helpers for async ───────────────────────────────────────

    use embedded_io_async as _; // ensure the crate is available

    /// Internal helper: read from an async I2C register.
    async fn read_reg<I2C: AsyncI2c>(
        motor: &mut Motor<I2C>,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), MotorError> {
        motor
            .i2c
            .write_read(motor.addr, reg, buf)
            .await
            .map_err(|_| MotorError::I2c)
    }

    /// Internal helper: write to an async I2C register.
    async fn write_reg<I2C: AsyncI2c>(
        motor: &mut Motor<I2C>,
        reg: u8,
        data: &[u8],
    ) -> Result<(), MotorError> {
        motor
            .i2c
            .write(motor.addr, reg, data)
            .await
            .map_err(|_| MotorError::I2c)
    }

    async fn read_u8<I2C: AsyncI2c>(motor: &mut Motor<I2C>, reg: u8) -> Result<u8, MotorError> {
        let mut buf = [0u8; 1];
        read_reg(motor, reg, &mut buf).await?;
        Ok(buf[0])
    }

    async fn write_u8<I2C: AsyncI2c>(
        motor: &mut Motor<I2C>,
        reg: u8,
        val: u8,
    ) -> Result<u8, MotorError> {
        write_reg(motor, reg, &[val]).await?;
        Ok(val)
    }

    async fn read_i16_le<I2C: AsyncI2c>(
        motor: &mut Motor<I2C>,
        reg: u8,
    ) -> Result<i16, MotorError> {
        let mut buf = [0u8; 2];
        read_reg(motor, reg, &mut buf).await?;
        Ok(i16::from_le_bytes(buf))
    }

    async fn read_u24_le<I2C: AsyncI2c>(
        motor: &mut Motor<I2C>,
        reg: u8,
    ) -> Result<u32, MotorError> {
        let mut buf = [0u8; 3];
        read_reg(motor, reg, &mut buf).await?;
        Ok(u32::from_le_bytes([buf[0], buf[1], buf[2], 0]))
    }

    async fn check_address<I2C: AsyncI2c>(motor: &mut Motor<I2C>, addr: u8) -> bool {
        motor.i2c.write(addr, 0x00, &[]).await.is_ok()
    }

    async fn scan_for_device<I2C: AsyncI2c>(motor: &mut Motor<I2C>) -> bool {
        for i in 1..127 {
            if !check_address(motor, i).await {
                continue;
            }
            motor.addr = i;
            let mut id = [0u8; 4];
            if read_reg(motor, REG_MODEL, &mut id).await.is_err() {
                motor.addr = 0;
                continue;
            }
            if id[0] != DEF_MODEL_MOT {
                motor.addr = 0;
                continue;
            }
            let device_addr = id[2] >> 1;
            if device_addr != i && id[2] != 0xFF {
                motor.addr = 0;
                continue;
            }
            let chip_id = id[3];
            if chip_id != DEF_CHIP_ID_FLASH && chip_id != DEF_CHIP_ID_METRO {
                motor.addr = 0;
                continue;
            }
            motor.version = id[1];
            return true;
        }
        motor.addr = 0;
        false
    }

    async fn set_stop_revs<I2C: AsyncI2c>(
        motor: &mut Motor<I2C>,
        revs: f32,
    ) -> Result<(), MotorError> {
        let revs = revs.clamp(0.0, 167_772.15);
        let cent = (revs * 100.0) as u32;
        write_reg(motor, REG_MOT_STOP_REV_L, &cent.to_le_bytes()[..3]).await
    }

    // ── Trait implementation ─────────────────────────────────────────────

    impl<I2C: AsyncI2c> MotorAsyncExt for Motor<I2C> {
        async fn begin(&mut self) -> Result<(), MotorError> {
            if self.addr == 0 {
                if !scan_for_device(self).await {
                    return Err(MotorError::I2c);
                }
            }
            if !check_address(self, self.addr).await {
                self.addr = 0;
                return Err(MotorError::I2c);
            }
            let mut id = [0u8; 4];
            read_reg(self, REG_MODEL, &mut id).await?;
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
            self.reset().await?;
            Ok(())
        }

        async fn reset(&mut self) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let bits0 = read_u8(self, REG_BITS_0).await?;
            write_u8(self, REG_BITS_0, bits0 | BIT_SET_RESET).await?;
            loop {
                let flg = read_u8(self, REG_FLAGS_0).await?;
                if flg & FLG_RESET != 0 {
                    break;
                }
            }
            Ok(())
        }

        async fn change_address(&mut self, new_addr: u8) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let new_addr = if new_addr > 0x7F { new_addr >> 1 } else { new_addr };
            if new_addr == 0x00 || new_addr == 0x7F {
                return Err(MotorError::InvalidParam);
            }
            let bits0 = read_u8(self, REG_BITS_0).await?;
            write_u8(self, REG_BITS_0, (bits0 & !BIT_BLOCK_ADR) | BIT_SAVE_ADR_EN).await?;
            write_u8(self, REG_ADDRESS, (new_addr << 1) | 0x01).await?;
            if !check_address(self, new_addr).await {
                return Err(MotorError::I2c);
            }
            self.addr = new_addr;
            Ok(())
        }

        async fn has_pull_i2c(&mut self) -> Result<bool, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let flg = read_u8(self, REG_FLAGS_0).await?;
            Ok(flg & FLG_I2C_UP != 0)
        }

        async fn pull_i2c(&mut self) -> Result<bool, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let mut buf = [0u8; 2];
            read_reg(self, REG_FLAGS_0, &mut buf).await?;
            if buf[0] & FLG_I2C_UP == 0 {
                return Err(MotorError::Unsupported);
            }
            Ok(buf[1] & BIT_SET_I2C_UP != 0)
        }

        async fn set_pull_i2c(&mut self, enable: bool) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let mut buf = [0u8; 2];
            read_reg(self, REG_FLAGS_0, &mut buf).await?;
            if buf[0] & FLG_I2C_UP == 0 {
                return Err(MotorError::Unsupported);
            }
            let new_bits = if enable { buf[1] | BIT_SET_I2C_UP } else { buf[1] & !BIT_SET_I2C_UP };
            write_u8(self, REG_BITS_0, new_bits).await?;
            Ok(())
        }

        async fn set_pwm_freq(&mut self, freq_hz: u16) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            if !(25..=1000).contains(&freq_hz) {
                return Err(MotorError::InvalidParam);
            }
            write_reg(self, REG_MOT_FREQUENCY_L, &freq_hz.to_le_bytes()).await
        }

        async fn set_magnet_count(&mut self, count: u8) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            if count == 0 {
                return Err(MotorError::InvalidParam);
            }
            write_u8(self, REG_MOT_MAGNET, count).await?;
            Ok(())
        }

        async fn magnet_count(&mut self) -> Result<u8, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            read_u8(self, REG_MOT_MAGNET).await
        }

        async fn set_gear_ratio(&mut self, ratio: f32) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let ratio = ratio.clamp(0.01, 167_772.15);
            let cent = (ratio * 100.0) as u32;
            write_reg(self, REG_MOT_REDUCER_L, &cent.to_le_bytes()[..3]).await
        }

        async fn gear_ratio(&mut self) -> Result<f32, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let raw = read_u24_le(self, REG_MOT_REDUCER_L).await?;
            Ok(raw as f32 / 100.0)
        }

        async fn set_error_threshold(&mut self, pct: u8) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            write_u8(self, REG_MOT_MAX_RPM_DEV, pct.min(100)).await?;
            Ok(())
        }

        async fn error_flags(&mut self) -> Result<ErrorFlags, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let flg = read_u8(self, REG_MOT_FLG).await?;
            if flg & MOT_FLG_RPM_ERR != 0 {
                Ok(ErrorFlags::SpeedError)
            } else if flg & MOT_FLG_DRV_ERR != 0 {
                Ok(ErrorFlags::DriverError)
            } else {
                Ok(ErrorFlags::None)
            }
        }

        async fn set_speed_rpm(&mut self, rpm: i16) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            write_reg(self, REG_MOT_SET_RPM_L, &rpm.to_le_bytes()).await
        }

        async fn set_speed_pwm_raw(&mut self, pwm: i16) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let pwm = pwm.clamp(-4095, 4095);
            write_reg(self, REG_MOT_SET_PWM_L, &pwm.to_le_bytes()).await
        }

        async fn set_speed_pwm_pct(&mut self, pct: f32) -> Result<(), MotorError> {
            let pct = pct.clamp(-100.0, 100.0);
            let pwm = (pct * 4095.0 / 100.0) as i16;
            self.set_speed_pwm_raw(pwm).await
        }

        async fn set_speed_m_per_s(&mut self, mps: f32) -> Result<(), MotorError> {
            let mm_per_min = mps * 60_000.0;
            let circumference_mm = 2.0 * PI * self.radius_mm;
            if circumference_mm <= f32::EPSILON {
                return Err(MotorError::InvalidParam);
            }
            let rpm = (mm_per_min / circumference_mm) as i16;
            self.set_speed_rpm(rpm).await
        }

        async fn set_speed_with_stop(
            &mut self,
            speed_val: f32,
            speed_type: SpeedType,
            stop_val: f32,
            stop_type: StopType,
        ) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            match stop_type {
                StopType::Immediate => {
                    let flg = read_u8(self, REG_MOT_FLG).await?;
                    let neutral = if flg & MOT_FLG_NEUTRAL != 0 { MOT_BIT_NEUTRAL } else { 0 };
                    write_u8(self, REG_MOT_STOP, MOT_BIT_STOP | neutral).await?;
                }
                StopType::Meters => {
                    let dist_mm = stop_val * 1000.0;
                    let circumference_mm = 2.0 * PI * self.radius_mm;
                    if circumference_mm <= f32::EPSILON {
                        return Err(MotorError::InvalidParam);
                    }
                    set_stop_revs(self, dist_mm / circumference_mm).await?;
                }
                StopType::Revolutions => {
                    set_stop_revs(self, stop_val).await?;
                }
                StopType::Seconds => {
                    let ms = (stop_val * 1000.0) as u32;
                    let ms = ms.min(16_777_215);
                    write_reg(self, REG_MOT_STOP_TMR_L, &ms.to_le_bytes()[..3]).await?;
                }
            }
            match speed_type {
                SpeedType::Rpm => self.set_speed_rpm(speed_val as i16).await,
                SpeedType::Pwm => self.set_speed_pwm_pct(speed_val).await,
                SpeedType::MPerSec => self.set_speed_m_per_s(speed_val).await,
            }
        }

        async fn speed(&mut self, speed_type: SpeedType) -> Result<f32, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let (reg, is_pwm) = match speed_type {
                SpeedType::Rpm | SpeedType::MPerSec => (REG_MOT_GET_RPM_L, false),
                SpeedType::Pwm => (REG_MOT_SET_PWM_L, true),
            };
            let raw = read_i16_le(self, reg).await?;
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

        async fn stop(&mut self) -> Result<(), MotorError> {
            let flg = read_u8(self, REG_MOT_FLG).await?;
            let neutral = if flg & MOT_FLG_NEUTRAL != 0 { MOT_BIT_NEUTRAL } else { 0 };
            write_u8(self, REG_MOT_STOP, MOT_BIT_STOP | neutral).await?;
            Ok(())
        }

        async fn set_stop(&mut self, val: f32, stop_type: StopType) -> Result<(), MotorError> {
            match stop_type {
                StopType::Immediate => self.stop().await,
                StopType::Meters => {
                    let dist_mm = val * 1000.0;
                    let circumference_mm = 2.0 * PI * self.radius_mm;
                    if circumference_mm <= f32::EPSILON {
                        return Err(MotorError::InvalidParam);
                    }
                    set_stop_revs(self, dist_mm / circumference_mm).await
                }
                StopType::Revolutions => set_stop_revs(self, val).await,
                StopType::Seconds => {
                    let ms = (val * 1000.0) as u32;
                    let ms = ms.min(16_777_215);
                    write_reg(self, REG_MOT_STOP_TMR_L, &ms.to_le_bytes()[..3]).await
                }
            }
        }

        async fn remaining_stop(&mut self, stop_type: StopType) -> Result<f32, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let mut buf = [0u8; 6];
            read_reg(self, REG_MOT_STOP_REV_L, &mut buf).await?;
            let rev_raw = u32::from_le_bytes([buf[0], buf[1], buf[2], 0]);
            let revs = rev_raw as f32 / 100.0;
            let tmr_raw = u32::from_le_bytes([buf[3], buf[4], buf[5], 0]);
            let magnets = read_u8(self, REG_MOT_MAGNET).await?;
            if magnets == 0 && stop_type != StopType::Seconds {
                return Ok(0.0);
            }
            match stop_type {
                StopType::Meters => {
                    let result = if revs > 0.0 { revs } else if tmr_raw > 0 {
                        let rpm = read_i16_le(self, REG_MOT_GET_RPM_L).await?;
                        let rpm = rpm.unsigned_abs() as f32;
                        let rpm = if rpm == 0.0 { 1.0 } else { rpm };
                        rpm * tmr_raw as f32 / 60_000.0
                    } else { 0.0 };
                    let circumference_mm = 2.0 * PI * self.radius_mm;
                    Ok(result * circumference_mm / 1000.0)
                }
                StopType::Revolutions => {
                    if revs > 0.0 { Ok(revs) } else if tmr_raw > 0 {
                        let rpm = read_i16_le(self, REG_MOT_GET_RPM_L).await?;
                        let rpm = rpm.unsigned_abs() as f32;
                        let rpm = if rpm == 0.0 { 1.0 } else { rpm };
                        Ok(rpm * tmr_raw as f32 / 60_000.0)
                    } else { Ok(0.0) }
                }
                StopType::Seconds => {
                    if tmr_raw > 0 { Ok(tmr_raw as f32 / 1000.0) } else if revs > 0.0 {
                        let rpm = read_i16_le(self, REG_MOT_GET_RPM_L).await?;
                        let rpm = rpm.unsigned_abs() as f32;
                        let rpm = if rpm == 0.0 { 1.0 } else { rpm };
                        Ok(revs * 60.0 / rpm)
                    } else { Ok(0.0) }
                }
                StopType::Immediate => Ok(0.0),
            }
        }

        async fn set_stop_neutral(&mut self, enabled: bool) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let flg = read_u8(self, REG_MOT_FLG).await?;
            let neutral = if enabled { MOT_BIT_NEUTRAL } else { 0 };
            let stop = if flg & MOT_FLG_STOP != 0 { MOT_BIT_STOP } else { 0 };
            write_u8(self, REG_MOT_STOP, neutral | stop).await?;
            Ok(())
        }

        async fn stop_neutral(&mut self) -> Result<bool, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let flg = read_u8(self, REG_MOT_FLG).await?;
            Ok(flg & MOT_FLG_NEUTRAL != 0)
        }

        async fn set_direction(&mut self, ckw: bool) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let bits = read_u8(self, REG_BITS_2).await?;
            let new_bits = if ckw { bits | MOT_BIT_DIR_CKW } else { bits & !MOT_BIT_DIR_CKW };
            write_u8(self, REG_BITS_2, new_bits).await?;
            Ok(())
        }

        async fn direction(&mut self) -> Result<bool, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let bits = read_u8(self, REG_BITS_2).await?;
            Ok(bits & MOT_BIT_DIR_CKW != 0)
        }

        async fn set_inversion(
            &mut self,
            reducer_inv: bool,
            rotor_inv: bool,
        ) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let bits = read_u8(self, REG_BITS_2).await?;
            let new_bits = bits & !(MOT_BIT_INV_RDR | MOT_BIT_INV_PIN);
            let new_bits = new_bits
                | if reducer_inv { MOT_BIT_INV_RDR } else { 0 }
                | if rotor_inv { MOT_BIT_INV_PIN } else { 0 };
            write_u8(self, REG_BITS_2, new_bits).await?;
            Ok(())
        }

        async fn inversion(&mut self) -> Result<(bool, bool), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let bits = read_u8(self, REG_BITS_2).await?;
            Ok((bits & MOT_BIT_INV_RDR != 0, bits & MOT_BIT_INV_PIN != 0))
        }

        async fn total_revolutions(&mut self) -> Result<f32, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let raw = read_u24_le(self, REG_MOT_GET_REV_L).await?;
            Ok(raw as f32 / 100.0)
        }

        async fn total_distance_m(&mut self) -> Result<f32, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let revs = self.total_revolutions().await?;
            let circumference_mm = 2.0 * PI * self.radius_mm;
            Ok(revs * circumference_mm / 1000.0)
        }

        async fn reset_total(&mut self) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            write_reg(self, REG_MOT_STOP_REV_L, &[0x00, 0x00, 0x00]).await
        }

        async fn set_nominal_voltage(&mut self, volts: f32) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let volts = volts.clamp(0.0, 25.5);
            write_u8(self, REG_MOT_VOLTAGE, (volts * 10.0) as u8).await?;
            Ok(())
        }

        async fn nominal_voltage(&mut self) -> Result<f32, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let raw = read_u8(self, REG_MOT_VOLTAGE).await?;
            Ok(raw as f32 / 10.0)
        }

        async fn set_nominal_rpm(&mut self, rpm: u16) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            write_reg(self, REG_MOT_NOMINAL_RPM_L, &rpm.to_le_bytes()).await
        }

        async fn nominal_rpm(&mut self) -> Result<u16, MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let mut buf = [0u8; 2];
            read_reg(self, REG_MOT_NOMINAL_RPM_L, &mut buf).await?;
            Ok(u16::from_le_bytes(buf))
        }

        async fn save_to_flash(&mut self, code: u64) -> Result<(), MotorError> {
            if self.addr == 0 {
                return Err(MotorError::NotInitialized);
            }
            let bytes = code.to_le_bytes();
            write_reg(self, REG_MANUFACTURER, &bytes[..5]).await
        }
    }
}

#[cfg(feature = "async")]
pub use asyn::{AsyncI2c, MotorAsyncExt};
