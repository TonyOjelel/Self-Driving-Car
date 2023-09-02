# Self-Driving Car Arduino Code Documentation

This documentation provides an overview of the Arduino code used in our self-driving car project. The code is designed to control various components and sensors to enable autonomous navigation.

# Huskylens

The HuskyLens library is included, but you need to initialize it based on your HuskyLens setup. Uncomment the HuskyLens initialization line and adjust the Serial port accordingly.

WiFi and UDP communication are retained based on your previous code.

Pixy 2.0 and Arduino Nano-related code are removed.

HuskyLens-related code is included to handle object detection and color identification. You will need to adapt the code to your specific HuskyLens configuration and processing needs.

Make sure to install and import the HuskyLens library into your Arduino IDE before using this code.

# Car stopping after 3 laps
Additionally, the logic for stopping the car after 3 laps is included in the code

## Code Structure

The code is organized into multiple sections and functions to manage different aspects of the self-driving car. Here's an overview of the code structure:

1. **Libraries and Definitions**:
   - The code includes necessary libraries and defines various constants and pins used throughout the program.

2. **Initialization**:
   - The setup function initializes serial communication, pins, sensors, and other components.

3. **Main Loop**:
   - The main loop continuously reads data from serial communication, updates sensor values, and manages the car's navigation.

4. **Direction Error Calculation**:
   - A function calculates the direction error between the car's current bearing and the objective direction.

5. **Speed and Steering Control**:
   - Functions control the car's speed and steering angle based on the calculated direction error.
   - These values are sent over serial communication to control the car's movement.

6. **Data Reception**:
   - The `receiveData` function handles incoming data, including encoder measurements and tension values.
   - It updates the car's status based on received data.

7. **Tension Management**:
   - The `manageTension` function manages the car's battery tension status and controls LED indicators accordingly.

8. **Ultrasonic Sensors and Huskylens Data Collection Task**:
   - A separate task named `UltrasonicHuskylensTaskCode` manages data collection from ultrasonic sensors and Huskylens camera.
   - It records object detection and color identification data from the Huskylens camera, along with ultrasonic sensor data.

## Sensor Integration

The code integrates several sensors and components into the self-driving car system:

- **MPU 6050 Gyroscope Sensors**: Used for stability enhancement and tracking during navigation.

- **Ultrasonic Sensors (DF URM09)**: Utilized for measuring distances and detecting obstacles at the front of the car.

- **Huskylens Camera**: Provides object detection and color identification data for navigation and decision-making.

## Autonomous Navigation Logic

The code implements a logic flow for autonomous navigation, including decision-making, obstacle detection, and turning:

- The car transitions between states (e.g., "Recto," "DecidiendoGiro," "Girando") to determine its actions.

- It uses the Huskylens camera to detect objects and their colors, making decisions based on the camera's input.

- The car autonomously adjusts its steering angle and speed to navigate through predefined scenarios, such as detecting objects and making turns.

## Communication and Telemetry (Optional)

The code includes optional features for communication and telemetry:

- **Wi-Fi and Over-The-Air (OTA) Updates**: Enables wireless updates and communication with a remote monitoring system.

- **Telemetry Data**: Collects and transmits telemetry data, including sensor readings and car status, to a designated IP address and port.

## Conclusion

This Arduino code forms the core logic of the self-driving car, integrating various sensors and components for autonomous navigation. 


