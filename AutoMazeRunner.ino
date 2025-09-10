#include <Servo.h> // Bibliothèque pour contrôler les servomoteurs

// ----------------- Définition des broches pour le moteur -----------------
#define EN 3  // Broche d'activation (Enable) du moteur
#define IN1 5 // Broches de contrôle moteur 1
#define IN2 7
#define IN3 6 // Broches de contrôle moteur 2
#define IN4 8

// ----------------- Définition des constantes -----------------
#define speed_Max 100    // Vitesse maximale du moteur
#define Turn90deg 500    // Durée approximative pour tourner à 90 degrés

// ----------------- Servo moteur et capteur -----------------
Servo monServo; // Objet Servo pour contrôler le servomoteur

int TRIG = 13; // Broche TRIG du capteur ultrason
int ECHO = 12; // Broche ECHO du capteur ultrason

long LECTURE_ECHO; // Temps mesuré par le capteur ultrason
long CM;           // Distance en cm
long CM_gauche;    // Distance mesurée à gauche
long CM_droite;    // Distance mesurée à droite

// Distances finales utilisées pour la logique du robot
int distanceGauche = 0;
int distanceDroite = 0;

// ----------------- Fonction d'initialisation -----------------
void setup() {
  // Définir les broches moteurs comme sorties
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT); 

  // Activer le moteur avec la vitesse maximale
  analogWrite(EN, speed_Max);

  // Attacher le servomoteur à la broche 10
  monServo.attach(10);

  // Définir les broches du capteur ultrason
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Démarrer la communication série pour afficher les distances
  Serial.begin(9600);
}

// ----------------- Boucle principale -----------------
void loop() {
  // -------- Lecture du capteur ultrason --------
  digitalWrite(TRIG, HIGH);          // Envoie l'impulsion ultrason
  delayMicroseconds(1);              // Temps très court
  digitalWrite(TRIG, LOW);           // Fin de l'impulsion
  LECTURE_ECHO = pulseIn(ECHO, HIGH);// Lecture du signal de retour
  CM = LECTURE_ECHO / 58;            // Conversion du temps en distance cm
  Serial.println(CM);                // Affiche la distance mesurée

  // -------- Contrôle du servomoteur --------
  int position = 90;                  // Servomoteur centré
  monServo.write(position);

  // -------- Mouvement du robot --------
  moveStraight();                     // Avance tout droit

  // Si un obstacle est trop proche (< 30 cm)
  if(CM <= 30){
    stop();                           // Arrête le robot
    verifygauchedroite();             // Vérifie les distances gauche/droite
    delay(1000);                      // Pause avant de décider

    // Décision selon la distance la plus grande
    if(distanceGauche <= distanceDroite){
      turnRight();                    // Tourne à droite
      delay(350);                     // Temps approximatif pour tourner
    }
    else if(distanceGauche > distanceDroite){
      turnLeft();                     // Tourne à gauche
      delay(340);
    }
  }  
}

// ----------------- Fonctions de mouvements -----------------
void moveStraight() {
  // Active tous les moteurs pour avancer
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  // Logique pour tourner à droite (moteurs opposés)
  digitalWrite(IN1, HIGH);
  analogWrite(IN2, -1);
  analogWrite(IN3, -1);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  // Logique pour tourner à gauche (moteurs opposés)
  analogWrite(IN1, -1);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  analogWrite(IN4, -1);
}

void moveBackward() {
  // Recule le robot
  analogWrite(IN1, -1);
  analogWrite(IN2, -1);
  analogWrite(IN3, -1);
  analogWrite(IN4, -1);
}

void stop() {
  // Arrête tous les moteurs
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

// ----------------- Fonctions pour le capteur ultrason -----------------
void takemesuregauche() {
  // Mesure la distance à gauche
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(1);
  digitalWrite(TRIG, LOW);
  LECTURE_ECHO = pulseIn(ECHO, HIGH);
  CM_gauche = LECTURE_ECHO / 58;
  Serial.println(CM_gauche);
  return(CM_gauche);
}

void takemesuredroite() {
  // Mesure la distance à droite
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(1);
  digitalWrite(TRIG, LOW);
  LECTURE_ECHO = pulseIn(ECHO, HIGH);
  CM_droite = LECTURE_ECHO / 58;
  Serial.println(CM_droite);
  return(CM_droite);
}

// ----------------- Vérification des distances à gauche et à droite -----------------
void verifygauchedroite() {
  // Servomoteur orienté à gauche
  int positionGauche = 180;
  monServo.write(positionGauche);
  delay(300);
  takemesuregauche();             // Mesure distance gauche
  distanceGauche = CM_gauche;

  // Servomoteur orienté à droite
  int positionDroite = 0;
  monServo.write(positionDroite);
  delay(300);
  takemesuredroite();             // Mesure distance droite
  distanceDroite = CM_droite;
}
