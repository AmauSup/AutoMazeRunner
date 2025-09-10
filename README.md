Autonomous Maze-Solving Robot with Audio Detection

🧠 Project Overview

This project demonstrates the development of an autonomous robot capable of navigating and solving a maze using audio sensors to detect walls. The robot employs a 4-wheel drive system and utilizes an audio detection mechanism to sense obstacles, enabling it to make real-time navigation decisions.

⚙️ Hardware Components

Arduino Uno: Microcontroller for processing sensor data and controlling motors.

4WD Robot Chassis: Provides mobility for the robot.

L298N Motor Driver: Controls the motors based on signals from the Arduino.

Ultrasonic Sensors (3 units): Measures distances to detect obstacles.

Audio Sensors (Microphone Array): Detects sounds reflecting off walls to aid in navigation.

Servo Motor: Controls the orientation of the audio sensors.

Power Supply: Provides necessary power to all components.

📦 Software Requirements

Arduino IDE: Development environment for programming the Arduino.

NewPing Library: Efficient control of ultrasonic sensors.

Servo Library: Controls the servo motor for sensor orientation.

Custom Audio Processing Code: Interprets audio signals for navigation decisions.

🧭 Navigation Algorithm

The robot employs a combination of ultrasonic and audio sensors to navigate the maze:

Audio Detection: The microphone array detects sounds reflecting off walls, indicating the presence of obstacles.

Sensor Orientation: The servo motor adjusts the orientation of the audio sensors to scan different directions.

Distance Measurement: Ultrasonic sensors measure the distance to obstacles in the robot's path.

Decision Making: Based on sensor inputs, the robot decides to move forward, turn left, or turn right to navigate the maze.

🛠️ Installation and Setup

Assemble the Hardware: Mount the ultrasonic sensors and microphone array on the robot chassis. Connect the motors to the L298N driver and wire the components to the Arduino Uno.

Install the Arduino IDE: Download and install the Arduino IDE from https://www.arduino.cc/en/software

Install Required Libraries:

Open the Arduino IDE.

Go to Sketch > Include Library > Manage Libraries.

Search for and install the following libraries:

NewPing

Servo

Upload the Code:

Open the provided Arduino sketch in the IDE.

Select the correct board and port under Tools.

Click the Upload button to transfer the code to the Arduino.

Power the Robot: Connect the power supply to the robot and ensure all components are receiving power.

📐 Circuit Diagram

Note: Ensure all connections are secure and correctly placed as per the diagram.

📄 Code Explanation

```
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
Note: The code provided is a basic framework. Additional logic for audio detection and decision-making should be implemented based on your specific hardware and requirements.

📸 Project Images

📄 Conclusion

This project showcases the integration of audio sensors with traditional ultrasonic sensors to navigate a maze autonomously. By combining these technologies, the robot can make informed decisions about its environment, enhancing its ability to solve complex mazes.

For further development, consider implementing machine learning algorithms to improve the robot's decision-making process based on sensor inputs.
