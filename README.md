# IMU-Based Robot Motion Control

This repository contains two Arduino sketches for precise motion control of a two-wheeled differential drive robot using an **MPU6050 IMU** and an **L298N motor driver**.  

Both codes implement **closed-loop control** to improve accuracy and reliability in forward movement and turning.

---

## Features

- **Forward Motion (23 cm)**
  - Proportional correction using gyro feedback.
  - Motor bias calibration for drift compensation.
  - Soft start and soft stop for smoother motion.
  - Active braking (reverse thrust + short brake).
  - Debug output over serial for monitoring.

- **90° Turn**
  - PID-based turn control with derivative braking.
  - Predictive braking near the target angle.
  - Adjustable angle tolerance for precision.
  - Directional correction if overshoot occurs.
  - Serial monitoring of angle, error, and speed.

---

## Hardware Requirements

- Arduino-compatible microcontroller (ESP32 in code).
- L298N Motor Driver.
- MPU6050 IMU sensor.
- Two n20 motors with encoders.
- Power supply (battery pack or regulated DC).

---

## Pin Configuration

| Component         | Pin  |
|-------------------|------|
| ENA (Left PWM)    | 27   |
| IN1 (Left Dir)    | 25   |
| IN2 (Left Dir)    | 26   |
| ENB (Right PWM)   | 18   |
| IN3 (Right Dir)   | 19   |
| IN4 (Right Dir)   | 23   |
| I2C SDA           | 21   |
| I2C SCL           | 22   |

---

## Code Overview

### 1. Forward Movement (23 cm)

File: `forward_23cm.ino`

- Moves the robot forward ~23 cm.
- Uses **gyro-based correction** to maintain a straight path.
- Active braking ensures accurate stopping distance.

Key adjustable parameters:
- `BASE_SPEED` → Motor base speed.  
- `MOVE_DURATION` → Time to move (calibrated for ~23 cm).  
- `MOTOR_BIAS` → Adjust if robot drifts left/right.  
- `CORRECTION_FACTOR` → Controls proportional correction strength.  

---

### 2. Perfect 90° Turn

File: `turnright90.ino`

- Executes a **precise right turn of 90°**.
- Uses **PID control** on gyro data.
- Includes predictive braking to avoid overshoot.

Key adjustable parameters:
- `targetAngle` → Desired turn angle (90° by default).  
- `Kp, Kd` → PID tuning constants.  
- `brakingZone` → Start braking early for smoother stop.  
- `angleTolerance` → Precision window (default ±0.8°).  

---

## Usage

1. Clone this repository:
   ```bash
   git clone https://github.com/Discovir/maze_solver.git
