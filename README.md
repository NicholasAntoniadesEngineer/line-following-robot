# STM32 Line Following Car

This repository contains the code and documentation for a line-following car project using an STM32F0 microcontroller. The project is designed to demonstrate the capabilities of embedded systems in robotics applications.

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

## Getting Started

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/stm32-line-following-car.git
   ```

2. **Set Up the Hardware**: Assemble the car according to the hardware requirements.

3. **Configure the Software**:
   - Open the project in STM32CubeIDE.
   - Configure the pins and peripherals as per your hardware setup.

4. **Build and Flash**:
   - Compile the code.
   - Flash the firmware onto the STM32 microcontroller.

5. **Test the Car**: Place the car on a track and observe its line-following behavior.

