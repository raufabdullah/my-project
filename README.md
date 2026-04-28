# Self-Balancing Robot

Two-wheeled self-balancing robot using the STM32F3 Discovery board and Keyestudio KS0193 kit. Uses onboard IMU sensors with a complementary filter and PID controller running at 200 Hz.

## Hardware

- STM32F303VCT6 (STM32F3 Discovery Board)
- Keyestudio KS0193 Self-Balancing Robot Kit
- TB6612FNG motor driver (on Keyestudio shield)
- L3GD20 gyroscope (onboard, SPI1)
- LSM303DLHC accelerometer (onboard, I2C1)
- DC power supply for motors
- USB-Serial adapter for UART output

## Pin Connections

| Pin | Function |
|-----|----------|
| PA5, PA6, PA7 | SPI1 (gyroscope) |
| PE3 | Gyro chip select |
| PB6, PB7 | I2C1 (accelerometer) |
| PA15 | TIM2 CH1 - Motor A PWM |
| PA1 | TIM2 CH2 - Motor B PWM |
| PB0, PB1 | Motor A direction (IN1, IN2) |
| PB2, PB14 | Motor B direction (IN3, IN4) |
| PA2, PA3 | USART2 TX/RX |

## Project Structure

```
Core/
├── Inc/
│   ├── app_config.h      # All tuneable parameters (PID gains, timing, thresholds)
│   ├── app_control.h      # Control loop interface
│   ├── app_imu.h          # Gyro + accel driver interface
│   ├── app_motor.h        # Motor driver interface
│   ├── app_uart.h         # UART wrapper interface
│   └── main.h             # CubeMX-generated pin defines
└── Src/
    ├── app_control.c      # Complementary filter, PID, rotation state machine
    ├── app_imu.c          # L3GD20 (SPI) + LSM303DLHC (I2C) drivers + calibration
    ├── app_motor.c        # TB6612FNG motor control (PWM + direction GPIO)
    ├── app_uart.c         # USART2 print wrapper
    └── main.c             # Init, ISR callback, telemetry loop
```

## How It Works

1. Gyro and accelerometer are calibrated at startup (keep robot still)
2. TIM6 fires an interrupt at 200 Hz, calling the control loop
3. Complementary filter fuses gyro rate + accel angle into a tilt estimate
4. PID controller computes motor output from the tilt error
5. Motors drive into the direction of tilt to keep the robot upright
6. Telemetry (angle, PID output) is printed over UART at 10 Hz

## PID Tuning

Edit values in `app_config.h`:

```c
#define KP 35.0f
#define KI 0.20f
#define KD 0.25f
```

Tuning approach: set Ki=0, Kd=0, raise Kp until it oscillates, then raise Kd until oscillation damps out, then add small Ki if there's steady-state drift.

## Real-Time Plotting

```bash
pip install pyserial matplotlib
sudo python plot.py
```

Make sure no other process (like `screen`) is using the serial port.

## CubeMX Notes

- SPI1 must be **8-bit** data size (not 9-bit default), CPOL=High, CPHA=2Edge
- USART2 on PA2/PA3 (USART1 is not used)
- TIM6 global interrupt must be enabled in NVIC

## Authors

Rauf Abdullah & Ammar Rehman — Habib University, Spring 2026
