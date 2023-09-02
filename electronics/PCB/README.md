# Self-Driving Car Algorithm Documentation

## Introduction
This documentation outlines the algorithm that enables our self-driving car to navigate autonomously. The project utilizes two microcontrollers, ESP32 as the master and Arduino Nano as the slave, which communicate through an I2C interface. Additionally, the car incorporates a Pixy 2.0 smart camera for obstacle detection, which communicates with the ESP32 via an SPI interface.

## Algorithm Overview
The primary goal of our self-driving car is to navigate to specific coordinates autonomously while avoiding obstacles. To achieve this, we've developed an algorithm based on a PID controller, focusing on proportional (P) and derivative (D) terms. This algorithm is responsible for determining the angle at which the car should point to reach the desired position.

## Components and Their Roles
Before delving into the algorithm's details, let's briefly review the key components and their roles in the self-driving car system:

- **Microcontrollers**:
  - **ESP32 (Master)**: Responsible for overall control, decision-making, and communication with the Arduino Nano and Pixy 2.0 camera.
  - **Arduino Nano (Slave)**: Manages motor control, servos, sensors, and provides data to the ESP32.
  
- **Pixy 2.0 Smart Camera**:
  - Detects objects and their colors, aiding in obstacle detection. Communicates with the ESP32 to provide object information.

- **Ultrasonic Sensors**:
  - Four sensors provide distance measurements for the car's front, sides, and rear. Data influences navigation decisions.

## Algorithm Details
The core of our self-driving algorithm revolves around the PID controller, specifically utilizing the P and D terms. Here's how it works:

1. **Setpoint and Variables**:
   - **Setpoint**: The desired target position (coordinates) that the car should reach autonomously.
   - **Controlled Variable**: The car's current position.

2. **Error Calculation**:
   - The algorithm calculates the error, which is the difference between the desired setpoint and the actual position of the car.

3. **Proportional (P) Term**:
   - The proportional term computes how far the car is from the setpoint.
   - This term determines the initial correction angle for the car to point towards the desired position.

4. **Derivative (D) Term**:
   - The derivative term considers the rate of change of the error.
   - It helps to dampen oscillations and smoothes the car's path as it approaches the setpoint.

5. **Actuator Control**:
   - The actuator in our case is the direction servo of the car.
   - The algorithm calculates the angle at which the car should point based on the combined P and D terms.

6. **Coordinate System Reset**:
   - To ensure consistent navigation and obstacle avoidance, the coordinate system is reset.
   - The car maintains a fixed distance (e.g., 20 cm) from the origin (center of the section) to avoid obstacles.
   - This resetting can be accomplished through mathematical operations or considering wall distances.

## Conclusion
Our self-driving car's algorithm is designed to enable it to autonomously navigate to specific coordinates while avoiding obstacles. By utilizing a PID controller with proportional and derivative terms, the car can accurately determine its orientation to reach the desired position. The collaborative efforts of the ESP32, Arduino Nano, Pixy 2.0 camera, and ultrasonic sensors ensure seamless and safe navigation. Further implementation details and code specifics should be available in the project's codebase and documentation.