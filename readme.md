# STM32 Line Following Car (2018)

This repository contains the code and documentation for a line-following car project using an STM32 microcontroller. The project is designed to demonstrate the capabilities of embedded systems in robotics applications.

## Project Overview

The STM32 Line Following Car is a robotic vehicle that autonomously follows a line on the ground. It uses sensors to detect the line and adjusts its steering to stay on course. This project is ideal for learning about embedded systems, sensor integration, and control algorithms.

## Features

- **Line Detection**: Utilizes infrared sensors to detect the line.
- **Control Algorithm**: Implements a PID controller for smooth line following.
- **Modular Design**: Easy to modify and extend for additional features.
- **Real-time Processing**: Leverages the STM32's processing power for real-time decision making.

## Hardware Requirements

- STM32 Microcontroller (e.g., STM32F103)
- Infrared Sensors
- Motor Driver (e.g., L298N)
- DC Motors
- Chassis for the car
- Power Supply

## Software Requirements

- STM32CubeIDE or any compatible IDE
- STM32 HAL Library
- C/C++ Compiler

## Control System Overview

The STM32 Line Following Car is equipped with a sophisticated control system that enables it to autonomously follow a line on the ground. The control system is primarily based on infrared (IR) sensors and a state machine that manages the robot's behavior.

### IR Sensors

The car uses IR sensors to detect the line's position. These sensors are strategically placed to provide real-time feedback on the car's alignment with the line. The sensors work by emitting infrared light and detecting the reflection from the surface. When the car deviates from the line, the reflection changes, allowing the control system to adjust the car's direction accordingly.

### Motor Control

The control system adjusts the speed of the left and right motors to maintain the car's alignment with the line. This is achieved through the `stm32_pwm_set_speed()` function, which processes the ADC readings from the IR sensors and generates the appropriate PWM signals to control the motor speeds. By continuously adjusting the motor speeds, the car can smoothly follow the line.

### State Machine

The robot's behavior is managed by a state machine implemented in the `robot_state_machine()` function. This state machine handles various states such as braking, driving, soft start, and reversing, based on button inputs. Each state corresponds to a specific action, such as activating the brakes or reversing the car, and is displayed on the LCD screen for user feedback.

The combination of IR sensors, motor control, and a state machine allows the STM32 Line Following Car to effectively navigate along a line, demonstrating the capabilities of embedded systems in robotics applications.
