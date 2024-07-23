//! This is a platform-agnostic Rust driver for the TMP108 temperature sensor
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.ti.com/lit/gpn/tmp108

#![doc(html_root_url = "https://docs.rs/tmp108/latest")]
#![cfg_attr(not(test), no_std)]

use embedded_hal::i2c::I2c;

mod registers;
pub use registers::*;

/// TMP108 device driver
pub struct Tmp108<I2C: I2c> {
    /// The concrete I2C bus implementation
    i2c: I2C,

    /// The I2C address.
    addr: u8,
}

impl<I2C: I2c> Tmp108<I2C> {
    const CELSIUS_PER_BIT: f32 = 0.0625;

    /// Create a new TMP108 instance with default address
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: A0Connection::default().into(),
        }
    }

    /// Create a new TMP108 instance with A0 tied to GND
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: A0Connection::Gnd.into(),
        }
    }

    /// Create a new TMP108 instance with A0 tied to V+
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: A0Connection::Vplus.into(),
        }
    }

    /// Create a new TMP108 instance with A0 tied to SDA
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: A0Connection::Sda.into(),
        }
    }

    /// Create a new TMP108 instance with A0 tied to SCL
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: A0Connection::Scl.into(),
        }
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Read configuration register
    pub fn configuration(&mut self) -> Result<Configuration, I2C::Error> {
        let data = self.read(Register::Configuration)?;
        Ok(Configuration::from(u16::from_be_bytes(data)))
    }

    /// Set configuration register
    pub fn set_configuration(&mut self, config: Configuration) -> Result<(), I2C::Error> {
        let value: u16 = config.into();
        self.write(Register::Configuration, value.to_be_bytes())
    }

    /// Read temperature register
    pub fn temperature(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.read(Register::Temperature)?;
        Ok(self.to_celsius(i16::from_be_bytes(raw)))
    }

    /// Read temperature low limit register
    pub fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.read(Register::LowLimit)?;
        Ok(self.to_celsius(i16::from_be_bytes(raw)))
    }

    /// Set temperature low limit register
    pub fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = self.to_raw(limit);
        self.write(Register::LowLimit, raw.to_be_bytes())
    }

    /// Read temperature high limit register
    pub fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.read(Register::HighLimit)?;
        Ok(self.to_celsius(i16::from_be_bytes(raw)))
    }

    /// Set temperature low limit register
    pub fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = self.to_raw(limit);
        self.write(Register::HighLimit, raw.to_be_bytes())
    }

    fn read(&mut self, reg: Register) -> Result<[u8; 2], I2C::Error> {
        let mut bytes = [0; 2];
        self.i2c.write_read(self.addr, &[reg.into()], &mut bytes)?;
        Ok(bytes)
    }

    fn write(&mut self, reg: Register, value: [u8; 2]) -> Result<(), I2C::Error> {
        let mut data = [0; 3];

        data[0] = reg.into();
        data[1] = value[0];
        data[2] = value[1];

        self.i2c.write(self.addr, &data)
    }

    fn to_celsius(&self, t: i16) -> f32 {
        f32::from(t / 16) * Self::CELSIUS_PER_BIT
    }

    fn to_raw(&self, t: f32) -> i16 {
        (t * 16.0 / Self::CELSIUS_PER_BIT) as i16
    }
}

/// A0 pin logic level representation.
#[derive(Debug)]
enum A0Connection {
    /// A0 tied to GND (default).
    Gnd,
    /// A0 tied to V+.
    Vplus,
    /// A0 tied to SDA.
    Sda,
    /// A0 tied to SCL.
    Scl,
}

impl Default for A0Connection {
    fn default() -> Self {
        Self::Gnd
    }
}

impl From<A0Connection> for u8 {
    fn from(connection: A0Connection) -> Self {
        match connection {
            A0Connection::Gnd => 0b100_1000,
            A0Connection::Vplus => 0b100_1001,
            A0Connection::Sda => 0b100_1010,
            A0Connection::Scl => 0b100_1011,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    #[test]
    fn handle_a0_pin_accordingly() {
        let expectations = vec![];

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new(mock);
        assert_eq!(tmp.addr, 0x48);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_gnd(mock);
        assert_eq!(tmp.addr, 0x48);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_vplus(mock);
        assert_eq!(tmp.addr, 0x49);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_sda(mock);
        assert_eq!(tmp.addr, 0x4a);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_scl(mock);
        assert_eq!(tmp.addr, 0x4b);
        let mut mock = tmp.destroy();
        mock.done();
    }

    #[test]
    fn read_temperature_default_address() {
        let expectations = vec![
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x7f, 0xf0])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x64, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x4b, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x40])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0xff, 0xc0])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0xe7, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0xc9, 0x00])],
        ];
        let temps = vec![
            127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0,
        ];

        for (e, t) in expectations.iter().zip(temps.iter()) {
            let mock = Mock::new(e);
            let mut tmp = Tmp108::new(mock);
            let result = tmp.temperature();
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_approx_eq!(temp, *t, 1e-4);

            let mut mock = tmp.destroy();
            mock.done();
        }
    }

    #[test]
    fn read_write_configuration_register() {
        let expectations = vec![
            Transaction::write_read(0x48, vec![0x01], vec![0x10, 0x22]),
            Transaction::write(0x48, vec![0x01, 0xb0, 0xfe]),
        ];

        let mock = Mock::new(&expectations);
        let mut tmp = Tmp108::new(mock);
        let result = tmp.configuration();
        assert!(result.is_ok());

        let cfg = result.unwrap();
        assert_eq!(cfg, Default::default());

        let cfg = cfg
            .with_cm(ConversionMode::Continuous)
            .with_tm(ThermostatMode::Interrupt)
            .with_fl(true)
            .with_fh(true)
            .with_cr(ConversionRate::Hertz16)
            .with_id(true)
            .with_hysteresis(Hysteresis::FourCelsius)
            .with_polarity(Polarity::ActiveHigh);

        let result = tmp.set_configuration(cfg);
        assert!(result.is_ok());

        let mut mock = tmp.destroy();
        mock.done();
    }
}
