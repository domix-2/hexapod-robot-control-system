 # **Gesture Controlled Hexapod Robot**

Six-legged mobile robot with mobile application control and hand gesture recognition using computer vision.

This project was developed as part of a Master's degree project in Mechanical Engineering (mag.ing.mech) and integrates robotics, embedded systems, and computer vision into a single robotic platform.
![Snimka zaslona 2026-02-28 222957](https://github.com/user-attachments/assets/25c17561-59d1-496e-8e17-213bb728b53d)

**Project Overview**

The goal of this project was to design and build a hexapod walking robot capable of being controlled in two ways:

📱 Mobile application via Bluetooth

✋ Hand gesture recognition using computer vision

The robot uses inverse kinematics to calculate leg movement and maintain stable walking.

**System Architecture**

The robot uses two main controllers:

- Raspberry Pi 5
- Camera processing
- Hand gesture recognition
- High-level control
- Arduino Nano ESP32
- Servo motor control
- IMU data processing
- Bluetooth communication
  
Communication between subsystems allows real-time control and sensor feedback.

**Hardware Components**

Main components used in the robot:

- Raspberry Pi 5
- Arduino Nano ESP32
- 2 × PCA9685 servo controllers
- 18 × MG996R servo motors
- LSM6DS3 IMU sensor
- HC-05 Bluetooth module
- Raspberry Pi Camera Module
- 3 × 18650 Li-ion batteries
- Buck converters for voltage regulation
- Microswitches for ground contact detection (possible expansion of robot movement)

![1000014631](https://github.com/user-attachments/assets/3cdeeb32-c403-4886-8f11-b5696e57c6bf)

**Robot Features**

- Six-legged walking robot
- Inverse kinematics based leg movement
- Stable walking gait
- Bluetooth control through mobile application
- Hand gesture recognition using computer vision
- Real-time sensor data monitoring
- Modular control architecture

## Author

Domagoj Špeljko  
Mechanical Engineer (mag.ing.mech)

Master's degree project focused on robotics, embedded systems and computer vision.

## License

This project is released for educational and research purposes.
