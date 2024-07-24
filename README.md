# TMP108

A `#[no_std]` platform-agnostic driver for the
[TMP108](https://www.ti.com/lit/gpn/tmp108) temperature sensor using
the [embedded-hal](https://docs.rs/embedded-hal) traits.

The TMP108 can take one of 4 I2C addresses depending on the state of
the A0 pin, as described in the table below:

| A0  | Addr |
|-----|------|
| GND | 0x48 |
| V+  | 0x49 |
| SDA | 0x4a |
| SCL | 0x4b |

The driver has specific constructors for each of these states, to
ensure that an invalid address is not attempted.

## Usage

```rust
let mut tmp = Tmp108::new(i2c);
// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
// let mut tmp = Tmp108::new_with_a0_vplus(i2c);
// let mut tmp = Tmp108::new_with_a0_sda(i2c);
// let mut tmp = Tmp108::new_with_a0_scl(i2c);

let cfg = Default::default()
    .with_cm(ConversionMode::OneShot)
    .with_tm(ThermostatMode::Comparator)
    .with_cr(ConversionRate::Hertz16)
    .with_hysteresis(Hysteresis::FourCelsius)
    .with_polarity(Polarity::ActiveLow);

tmp.set_configuration(cfg)?;

let temp = tmp.temperature()?;

let cfg = cfg
    .with_cm(ConversionMode::OneShot)
    .with_tm(ThermostatMode::Interrupt)
    .with_cr(ConversionRate::Hertz1)
    .with_fl(true)
    .with_fh(true);

tmp.set_configuration(cfg)?;

let high_limit = 48.0;
let low_limit = 26.5;

tmp.set_low_limit(low_limit)?;
tmp.set_high_limit(high_limit)?;

loop {
    let temp = tmp.temperature()?;
}
```

## MSRV

Currently, rust `1.79` and up is supported, but some previous versions
may work.

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

License: MIT

