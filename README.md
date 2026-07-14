# SelfBalanceRobot

Two-wheeled self-balancing robot built on the ESP32 (ESP-IDF), using an MPU6050 IMU for tilt sensing and a TB6612FNG dual H-bridge driver for motor control.

## Overview

The robot estimates its tilt angle from the IMU, fuses it with a complementary filter, and feeds the result into a PID controller that drives two DC motors to keep the robot upright.

**Control loop:**
```
MPU6050 (I2C) → Complementary Filter → PID Controller → TB6612FNG (MCPWM) → DC Motors
```

## Hardware

| Component        | Details                                |
|-------------------|-----------------------------------------|
| MCU               | ESP32 (ESP-IDF framework)               |
| IMU               | MPU6050 (accelerometer + gyroscope, I2C)|
| Motor driver      | TB6612FNG dual H-bridge                 |
| Motors            | 2x DC motors                            |
| PWM generation    | ESP32 MCPWM peripheral, 10 kHz          |

### Pin Configuration

**Motor A**
- PWM: GPIO 25
- IN1: GPIO 26
- IN2: GPIO 27

**Motor B**
- PWM: GPIO 14
- IN1: GPIO 32
- IN2: GPIO 33

**Shared**
- STBY: GPIO 4

**IMU (I2C)**
- SDA: GPIO 21
- SCL: GPIO 22
- I2C address: 0x68
- Clock: 400 kHz

## How It Works

1. **IMU reading** — Raw accelerometer and gyroscope data are read from the MPU6050 over I2C at ~100 Hz.
2. **Angle estimation** — A complementary filter (α = 0.98) combines the gyro's integrated rate with the accelerometer's absolute tilt angle to produce a stable, drift-corrected angle estimate.
3. **Gyro bias calibration** — On startup, the robot must be held still while ~300 gyro samples are averaged to estimate and cancel the sensor's zero-rate bias, reducing initial drift.
4. **PID control** — The filtered angle (setpoint = 0°, i.e. upright) drives a PID controller with anti-windup (clamped integral term) to compute a motor duty cycle.
5. **Safety cutoff** — If the tilt angle exceeds ±45°, the robot is considered fallen: both motors are cut to zero and the PID's integral/derivative history is reset, so the controller doesn't spike when the robot is picked back up.
6. **Motor output** — MCPWM comparators translate the PID output into direction (forward/backward via IN1/IN2) and duty cycle (0–100%) for each motor.

## Building & Flashing

Standard ESP-IDF workflow:

```bash
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

