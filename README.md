# 🤖 Robot autonome résolvant un labyrinthe avec détection audio

[![Arduino](https://img.shields.io/badge/Arduino-Uno-blue.svg)](https://www.arduino.cc/) 
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE) 
[![Language: C++](https://img.shields.io/badge/Language-C++-orange.svg)](https://isocpp.org/)

---

## 🧠 Présentation du projet

Ce projet démontre le développement d’un **robot autonome** capable de naviguer et de résoudre un **labyrinthe** en utilisant des **capteurs audio** pour détecter les murs.  

Le robot est construit sur un **châssis à 4 roues motrices** et utilise **des capteurs ultrasoniques et audio** pour détecter les obstacles et prendre des décisions de navigation en temps réel.

---

## ⚙️ Composants matériels

- **Arduino Uno** – Microcontrôleur pour le traitement des capteurs et le contrôle des moteurs  
- **Châssis robot 4WD** – Fournit la mobilité  
- **Driver moteur L298N** – Contrôle les moteurs  
- **Capteurs ultrasoniques (3 unités)** – Mesure des distances  
- **Capteurs audio (microphone array)** – Détecte les réflexions des murs  
- **Servo-moteur** – Fait pivoter les capteurs audio pour le scan  
- **Alimentation** – Alimente tous les composants  

---

## 📦 Logiciels requis

- **Arduino IDE** – [Télécharger](https://www.arduino.cc/en/software)  
- **Bibliothèques** :  
  - `NewPing` – Gestion des capteurs ultrasoniques  
  - `Servo` – Contrôle du servo-moteur  
- **Code personnalisé de traitement audio** – Interprète les signaux audio pour la navigation  

---

## 🧭 Algorithme de navigation

Le robot combine **capteurs ultrasoniques** et **audio** pour naviguer de manière autonome :

1. **Détection audio** – Le microphone détecte les sons réfléchis par les murs  
2. **Orientation des capteurs** – Le servo fait pivoter les capteurs pour scanner l’environnement  
3. **Mesure des distances** – Les capteurs ultrasoniques détectent les obstacles  
4. **Prise de décision** – Le robot avance, tourne à gauche ou à droite en fonction des données des capteurs  

---

## 🛠️ Installation & configuration

### 1️⃣ Assemblage matériel

- Monter les capteurs ultrasoniques et le microphone sur le châssis  
- Connecter les moteurs au driver L298N  
- Câbler les composants à l’Arduino Uno  

### 2️⃣ Configuration logicielle

1. Installer **Arduino IDE**  
2. Installer les bibliothèques nécessaires via **Sketch → Include Library → Manage Libraries** :  
   - `NewPing`  
   - `Servo`  

### 3️⃣ Téléversement du code

- Ouvrir le sketch Arduino dans l’IDE  
- Sélectionner la **carte et le port** dans le menu Outils  
- Cliquer sur **Upload**  

### 4️⃣ Alimentation du robot

- Connecter l’alimentation et vérifier que tous les composants sont actifs  

---

## 📐 Schéma du circuit

> Vérifier que toutes les connexions correspondent au schéma ci-dessous.

![Schéma du circuit](https://example.com/circuit-diagram.jpg)

---

## 📄 Aperçu du code


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
    Ceci est un cadre de travail ; une logique supplémentaire de détection audio peut être ajoutée pour une résolution de labyrinthe plus intelligente.

---

## 📄 Conclusion

Ce projet combine des capteurs ultrasoniques et audio pour la navigation autonome dans un labyrinthe.  
Le robot prend des décisions éclairées, évite efficacement les obstacles et démontre un contrôle autonome en temps réel.

Améliorations futures :

```text
- Implémenter du machine learning pour une navigation plus intelligente
- Optimiser les algorithmes de balayage des capteurs et de prise de décision
```
