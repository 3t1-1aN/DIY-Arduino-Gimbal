# DIY Arduino Gimbal

**A 3D-printed gimbal platform stabilized using an Arduino with MPU-6050 gyroscope-accelerometer for dynamic motion control.**

![Arduino Gimbal Screenshot](https://github.com/user-attachments/assets/ddef42ae-16ce-4202-9915-f62fda0016c0)

## Project Overview

This project implements a DIY 3D-printed gimbal system that uses an Arduino microcontroller and an MPU-6050 inertial measurement unit (IMU) to maintain a stable platform. The gimbal automatically compensates for movement and orientation changes, keeping mounted equipment level regardless of the base's motion.

Inspired by Thingiverse designs, this gimbal combines mechanical 3D-printed parts with electronic stabilization to create a functional platform stabilization system suitable for cameras, sensors, or other payloads.

## Key Features

- **3D-printed mechanical structure** for lightweight and customizable design
- **MPU-6050 IMU sensor** for real-time motion detection (6-axis: 3-axis gyroscope + 3-axis accelerometer)
- **Arduino-based control system** for processing sensor data and controlling actuators
- **Active stabilization** that compensates for pitch, roll, and yaw movements
- **Servo motor integration** for controlled axis movement
- **Real-time sensor fusion** for accurate orientation estimation

## Hardware Components

- **Microcontroller**: Arduino (Uno, Nano, or compatible)
- **IMU Sensor**: MPU-6050 (6-axis gyroscope-accelerometer)
- **Actuators**: Servo motors for multi-axis control
- **Structural Components**: 3D-printed parts
- **Power Supply**: USB or battery (depends on servo requirements)
- **Connecting Hardware**: Servo mounts, bearings, and mechanical linkages

## How It Works

The MPU-6050 sensor continuously monitors the gimbal's orientation and acceleration. The Arduino reads this sensor data and calculates the current orientation using sensor fusion algorithms. When the gimbal detects movement, the control system sends correction signals to the servo motors, which adjust the platform to maintain its original level orientation.

### Control Flow:

1. **Sensor Input**: MPU-6050 provides 6-axis motion data
2. **Data Processing**: Arduino calculates orientation (pitch, roll, yaw)
3. **Error Calculation**: Compares current vs. desired orientation
4. **Motor Control**: Adjusts servo positions via PWM signals
5. **Stabilization**: Platform remains stable despite base movement



https://github.com/user-attachments/assets/55923560-399f-4907-84b0-334c90e43fc7



## Technical Specifications

- **Sensor Interface**: I2C communication with MPU-6050
- **Motion Range**: Dependent on servo mechanical limits
- **Update Rate**: Real-time sensor polling at 100+ Hz
- **Gyroscope Range**: ±250°/s to ±2000°/s (configurable)
- **Accelerometer Range**: ±2g to ±16g (configurable)
- **Servo Control**: PWM-based position control

## Project Status

The gimbal successfully stabilizes a mounted platform using closed-loop feedback control. The system maintains orientation compensation across all three axes in real-time.

## Getting Started

### Prerequisites
- Arduino IDE or compatible development environment
- MPU-6050 library for Arduino
- Servo control library
- 3D printer or access to 3D printing services

### Assembly

1. Print all 3D components from the design files
2. Assemble mechanical parts with appropriate fasteners
3. Mount servo motors at the three rotation axes
4. Connect MPU-6050 to Arduino via I2C (SDA/SCL pins)
5. Connect servo control pins to Arduino PWM outputs
6. Upload calibrated firmware to the Arduino

## Notes

Proper calibration of the MPU-6050 is critical for accurate stabilization. Gyroscope offset calibration should be performed with the gimbal stationary before operation.
