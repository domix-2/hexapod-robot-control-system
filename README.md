#Gesture Controlled Hexapod Robot

Six-legged mobile robot with mobile application control and hand gesture recognition using computer vision.

This project was developed as part of a Master's degree project in Mechanical Engineering (mag.ing.mech) and integrates robotics, embedded systems, and computer vision into a single robotic platform.
![Snimka zaslona 2026-02-28 222957](https://github.com/user-attachments/assets/25c17561-59d1-496e-8e17-213bb728b53d)

Project Overview

The goal of this project was to design and build a hexapod walking robot capable of being controlled in two ways:

📱 Mobile application via Bluetooth

✋ Hand gesture recognition using computer vision

The robot uses inverse kinematics to calculate leg movement and maintain stable walking.

System Architecture

The robot uses two main controllers:

Raspberry Pi 5

Camera processing

Hand gesture recognition

High-level control

Arduino Nano ESP32

Servo motor control

IMU data processing

Bluetooth communication

Communication between subsystems allows real-time control and sensor feedback.
