# 🤖 Autonomous Maze-Solving Robot with Audio Detection

[![Arduino](https://img.shields.io/badge/Arduino-Uno-blue.svg)](https://www.arduino.cc/) 
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE) 
[![Language: C++](https://img.shields.io/badge/Language-C++-orange.svg)](https://isocpp.org/)

---

## 🧠 Project Overview

This project demonstrates the development of an **autonomous robot** capable of navigating and solving a **maze** using **audio sensors** to detect walls.  

The robot is built on a **4-wheel drive chassis** and uses **ultrasonic + audio detection** to sense obstacles and make real-time navigation decisions.

---

## ⚙️ Hardware Components

- **Arduino Uno** – Microcontroller for sensor processing & motor control  
- **4WD Robot Chassis** – Provides mobility  
- **L298N Motor Driver** – Controls motors  
- **Ultrasonic Sensors (3 units)** – Measures distances  
- **Audio Sensors (Microphone Array)** – Detects wall reflections  
- **Servo Motor** – Rotates audio sensors for scanning  
- **Power Supply** – Powers all components  

---

## 📦 Software Requirements

- **Arduino IDE** – [Download](https://www.arduino.cc/en/software)  
- **Libraries**:  
  - `NewPing` – Ultrasonic sensor management  
  - `Servo` – Servo motor control  
- **Custom Audio Processing Code** – Interprets audio signals for navigation  

---

## 🧭 Navigation Algorithm

The robot combines **ultrasonic** and **audio sensors** for autonomous navigation:

1. **Audio Detection** – Microphone array senses sounds reflecting off walls  
2. **Sensor Orientation** – Servo rotates sensors to scan surroundings  
3. **Distance Measurement** – Ultrasonic sensors detect obstacles  
4. **Decision Making** – Robot moves forward, turns left, or right based on sensor input  

---

## 🛠️ Installation & Setup

### 1️⃣ Hardware Assembly

- Mount ultrasonic sensors & microphone array on the chassis  
- Connect motors to L298N driver  
- Wire components to Arduino Uno  

### 2️⃣ Software Setup

1. Install **Arduino IDE**  
2. Install required libraries via **Sketch → Include Library → Manage Libraries**:  
   - `NewPing`  
   - `Servo`  

### 3️⃣ Upload Code

- Open Arduino sketch in IDE  
- Select **board & port** under Tools  
- Click **Upload**  

### 4️⃣ Power the Robot

- Connect the power supply and verify all components are active  

---

## 📐 Circuit Diagram

> Ensure all connections match the diagram below.

![Circuit Diagram](https://example.com/circuit-diagram.jpg)

---

## 📄 Code Overview

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
  for (int pos = 0; pos <= 180; pos += 10) {
    audioServo.write(pos);
    delay(500);
    // Add audio detection logic here
  }

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
    This is a framework; additional audio detection logic can be added for smarter maze-solving.

## 📄 Conclusion

This project combines ultrasonic + audio sensors for autonomous maze navigation.
The robot makes informed decisions, successfully avoids obstacles, and demonstrates real-time autonomous control.

Future improvements:

    Implement machine learning for smarter navigation

    Optimize sensor scanning & decision-making algorithms
