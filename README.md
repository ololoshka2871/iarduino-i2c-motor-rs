# iarduino-i2c-motor-rs

Rust драйвер для I2C контроллеров моторов iarduino (серия FLASH-I2C).

Исходная Arduino библиотека: [tremaru/iarduino_I2C_Motor](https://github.com/tremaru/iarduino_I2C_Motor) (v1.1.7).

__Поддерживаемые модули__:

| Модель                                                                                                            | Ссылка на магазин                                                                                                          |
| ----------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| <p></p> <img src="https://wiki.iarduino.ru/img/resources/1297/1297.svg" width="100px"></img><p> С энкодером </p>  | https://iarduino.ru/shop/Mehanika/motor-reduktor-n20-500rpm-s-upravlyayuschim-kontrollerom-flash-i2c.html                  |
| <p></p> <img src="https://wiki.iarduino.ru/img/resources/1402/1402.svg" width="100px"></img><p> Без энкодера </p> | https://iarduino.ru/shop/Mehanika/motor-reduktor-bez-enkodera-n20-100rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html |

## Возможности

- `no_std`, embedded-first
- Sync API через `embedded_hal::i2c::I2c` (всегда доступен)
- Async API через `MotorAsyncExt` trait (feature `async`, `embedded-io-async`)
- Ошибки через `MotorError` (impl `embedded_io::Error`)
- Управление скоростью: RPM, PWM, м/с
- Условия остановки: расстояние, обороты, время
- Обратная связь по датчику Холла
- Настройка редуктора, магнитов, напряжения, инверсии

## Установка

```toml
[dependencies]
iarduino-i2c-motor-rs = { git = "https://github.com/ololoshka2871/iarduino-i2c-motor-rs" }

# Для async:
iarduino-i2c-motor-rs = { git = "https://github.com/ololoshka2871/iarduino-i2c-motor-rs", features = ["async"] }
```

## Быстрый старт (sync)

```rust
use iarduino_i2c_motor_rs::{Motor, SpeedType, StopType};

// I2C шина (реализует embedded_hal::i2c::I2c)
let i2c = /* ... */;

let mut motor = Motor::new(i2c, 0x09);
// Или авто-поиск адреса: Motor::new_auto(i2c);

motor.begin()?;

// Скорость
motor.set_speed_rpm(1500)?;
motor.set_speed_pwm_pct(75.0)?;
motor.set_speed_m_per_s(0.5)?;

// Скорость + условие остановки
motor.set_speed_with_stop(1000.0, SpeedType::Rpm, 5.0, StopType::Seconds)?;

// Чтение скорости
let rpm = motor.speed(SpeedType::Rpm)?;
let pwm = motor.speed(SpeedType::Pwm)?;

// Остановка
motor.stop()?;
motor.set_stop(2.0, StopType::Meters)?;

// Оставшееся до остановки
let remaining = motor.remaining_stop(StopType::Seconds)?;

// Редуктор
motor.set_gear_ratio(50.0)?;   // 1:50
let ratio = motor.gear_ratio()?;

// Направление
motor.set_direction(true)?;    // по часовой при "+" скорости
let dir = motor.direction()?;

// Ошибки
let err = motor.error_flags()?;
```

## Async API

```rust
use iarduino_i2c_motor_rs::{Motor, MotorAsyncExt};

// I2C шина (реализует AsyncI2c)
let i2c = /* ... */;
let mut motor = Motor::new(i2c, 0x09);
motor.begin().await?;
motor.set_speed_rpm(1500).await?;
motor.stop().await?;
```

## Зависимости

| Крейт               | Версия                          |
| ------------------- | ------------------------------- |
| `embedded-hal`      | 1.0                             |
| `embedded-io`       | 0.7                             |
| `embedded-io-async` | 0.7 (optional, feature `async`) |

## Лицензия

MIT
