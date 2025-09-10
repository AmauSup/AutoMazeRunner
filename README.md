# 🤖 Autonomous Maze-Solving Robot with Audio Detection

---

## 🧠 Project Overview

This project demonstrates the development of an **autonomous robot** capable of navigating and solving a maze using **audio sensors** to detect walls.  
The robot is built on a **4-wheel drive chassis** and leverages an audio detection mechanism to sense obstacles, enabling **real-time navigation decisions**.

---

## ⚙️ Hardware Components

- **Arduino Uno** – Microcontroller for processing sensor data and controlling motors  
- **4WD Robot Chassis** – Provides mobility for the robot  
- **L298N Motor Driver** – Controls motors based on Arduino signals  
- **Ultrasonic Sensors (3 units)** – Measures distances to detect obstacles  
- **Audio Sensors (Microphone Array)** – Detects sounds reflecting off walls to aid in navigation  
- **Servo Motor** – Controls orientation of audio sensors  
- **Power Supply** – Provides necessary power to all components  

---

## 📦 Software Requirements

- **Arduino IDE** – Development environment for programming the Arduino  
- **NewPing Library** – Efficient ultrasonic sensor management  
- **Servo Library** – Controls the servo motor for sensor orientation  
- **Custom Audio Processing Code** – Interprets audio signals for navigation decisions  

---

## 🧭 Navigation Algorithm

The robot combines **ultrasonic** and **audio sensors** to navigate the maze:

1. **Audio Detection** – Microphone array detects sounds reflecting off walls.  
2. **Sensor Orientation** – Servo motor scans different directions for obstacle detection.  
3. **Distance Measurement** – Ultrasonic sensors measure proximity to obstacles.  
4. **Decision Making** – Based on sensor input, the robot moves forward, turns left, or turns right.  

---

## 🛠️ Installation and Setup

### 1. Assemble the Hardware

- Mount ultrasonic sensors and microphone array on the robot chassis  
- Connect motors to the L298N driver  
- Wire all components to the Arduino Uno  

### 2. Install Arduino IDE

Download from: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)  

### 3. Install Required Libraries

- Open Arduino IDE → **Sketch → Include Library → Manage Libraries**  
- Search and install:  
  - `NewPing`  
  - `Servo`  

### 4. Upload the Code

- Open the Arduino sketch in the IDE  
- Select the correct **board** and **port** under Tools  
- Click **Upload**  

### 5. Power the Robot

- Connect the power supply and ensure all components receive power  

---

## 📐 Circuit Diagram

> Ensure all connections are secure and placed correctly according to the diagram.

![Circuit Diagram](https://example.com/circuit-diagram.jpg)  

---

## 📄 Code Explanation

```cpp
#include <NewPing.h>
#include <Servo.h>

// Pin Definitions
#define TRIG_PIN 12
#define ECHO_PIN 13
#define SERVO_PIN 9
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 5
#define MOTOR_RIGHT_FORWARD 10
#define MOTOR_RIGHT_BACKWARD 11

// Constants
#define MAX_DISTANCE 200
#define TURN_DELAY 500

// Initialize Sensors and Servo
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo audioServo;

// Motor Control Functions
void moveForward() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  delay(TURN_DELAY);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  delay(TURN_DELAY);
}

// Setup Function
void setup() {
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  audioServo.attach(SERVO_PIN);
  Serial.begin(9600);
}

// Loop Function
void loop() {
  // Rotate audio sensor to scan surroundings
  for (int pos = 0; pos <= 180; pos += 10) {
    audioServo.write(pos);
    delay(500);
    // Add audio detection logic here
  }

  // Measure distance to obstacles
  int distance = sonar.ping_cm();
  if (distance > 0 && distance < 20) {
    moveBackward();
    delay(500);
    turnLeft();
  } else {
    moveForward();
  }
}
```
    This is a basic framework. Additional logic for audio detection and maze-solving strategies can be added based on your robot’s sensors.

## 📸 Project Images


📄 Conclusion

This project demonstrates how audio sensors can be combined with ultrasonic sensors to create an autonomous maze-solving robot.
The robot can detect obstacles and make real-time navigation decisions, showcasing the integration of hardware, software, and control algorithms.

Future improvements may include machine learning for smarter navigation and adaptive behavior.
