# MQTT and PID Control System on ESP32


## Overview
This project implements a **real-time closed-loop control system** on the **ESP32** using **FreeRTOS**, **MQTT communication**, and a **PID controller** to stabilize a servo actuator under random noise disturbances. The system simulates a realistic engineering control scenario where a mechanical actuator must remain stable despite unpredictable environmental noise, while publishing live telemetry data to an MQTT broker for remote monitoring.

## Objectives
- Implement a PID-controlled actuator system in real time  
- Use FreeRTOS to manage multiple concurrent tasks  
- Safely share data between tasks using synchronization primitives  
- Transmit sensor, noise, and actuator data via MQTT for visualization  

## System Architecture
The application is composed of multiple **FreeRTOS tasks** running concurrently on the ESP32:

### Core Tasks
- **Wi-Fi / MQTT Task**
  - Establishes and maintains Wi-Fi connectivity
  - Publishes sensor readings, noise values, PID output, and servo angles
  - Receives external setpoint updates from the MQTT broker
- **Noise Generation Task**
  - Continuously generates random noise to simulate environmental disturbances
- **Sensor Task**
  - Computes the effective sensor value by combining setpoint and noise
- **PID Controller Task (20 Hz)**
  - Calculates the control signal using proportional, integral, and derivative terms
  - Drives the system toward a target angle (90°)
- **Servo Actuator Task (20 Hz)**
  - Applies PID output to control the servo motor (simulated plant)
- **Status / Telemetry Task (1 Hz)**
  - Publishes system state, error metrics, and actuator position to MQTT

## Synchronization
- **Mutexes** are used to protect shared variables such as:
  - Noise values
  - Sensor readings
  - PID terms
  - Servo output
- Prevents race conditions and ensures deterministic real-time behavior

## Communication
- **MQTT Protocol** used for real-time data exchange
- Data published to a cloud broker (HiveMQ) for live monitoring
- System supports reconnection handling during Wi-Fi dropouts

## Results
- Stable real-time operation on ESP32
- Reliable MQTT publishing and reconnection behavior
- PID controller effectively stabilized the servo under moderate and high noise
- Minimal overshoot after tuning
- Derivative term reduced sudden spikes
- Integral limiting prevented wind-up
- All test cases passed successfully

## Key Concepts Demonstrated
- Real-time multitasking with FreeRTOS
- PID control and tuning
- Embedded synchronization using mutexes
- MQTT-based telemetry and remote monitoring
- Closed-loop control under noisy conditions

## Conclusion
This project successfully integrates **RTOS concepts**, **network communication**, and **PID control** into a single embedded application. It demonstrates how multiple synchronized tasks can cooperate to maintain system stability in the presence of noise, while providing real-time visibility through MQTT. The implementation meets all functional, stability, and performance requirements and reinforces core principles of embedded real-time control systems.
