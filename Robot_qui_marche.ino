#include <Servo.h>

// ----------------- Definition des fonctions

#define EN 3 
#define IN1 5 
#define IN2 7 
#define IN3 6 
#define IN4 8 

// -----------------



// ----------------- Variable
#define speed_Max 100
#define Turn90deg 500
// -----------------



// ----------------- Servo moteur / capteur
Servo monServo;

int TRIG = 13;
int ECHO = 12;

long LECTURE_ECHO;
long CM;
long CM_gauche;
long CM_droite;

// -----------------

int distanceGauche = 0;
int distanceDroite = 0;

void setup() {

pinMode(IN1, OUTPUT); 
pinMode(IN2, OUTPUT); 
pinMode(IN3, OUTPUT); 
pinMode(IN4, OUTPUT); 
analogWrite(EN, speed_Max);
monServo.attach(10);
pinMode(TRIG, OUTPUT);
pinMode(ECHO, INPUT);
Serial.begin(9600);

}

void loop() {

// ----------------- Capteur
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(1);
  digitalWrite(TRIG, LOW);
  LECTURE_ECHO = pulseIn(ECHO, HIGH);
  CM = LECTURE_ECHO / 58;
  Serial.println(CM);
// -----------------

int position = 90; 
monServo.write(position);
moveStraight();

if(CM <= 30){

  stop();
  verifygauchedroite();
  delay(1000);

    if(distanceGauche <= distanceDroite){

      turnRight();
      delay(350);

    }
    else if(distanceGauche >= distanceDroite){
      turnLeft();
      delay(340);
    }
  }
  
}


// ---------------- Fonctions enregistrée

//----------------- Mouvements
void moveStraight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}


void turnRight() {
  digitalWrite(IN1, HIGH);
  analogWrite(IN2, -1);
  analogWrite(IN3, -1);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
   analogWrite(IN1, -1);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
   analogWrite(IN4, -1);
}

void moveBackward() {
  analogWrite(IN1, -1);
  analogWrite(IN2, -1);
  analogWrite(IN3, -1);
  analogWrite(IN4, -1);
}

void stop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
//----------------- 

void takemesuregauche() {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(1);
  digitalWrite(TRIG, LOW);
  LECTURE_ECHO = pulseIn(ECHO, HIGH);
  CM_gauche = LECTURE_ECHO / 58;
  Serial.println(CM_gauche);
  return(CM_gauche);

}

void takemesuredroite() {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(1);
  digitalWrite(TRIG, LOW);
  LECTURE_ECHO = pulseIn(ECHO, HIGH);
  CM_droite = LECTURE_ECHO / 58;
  Serial.println(CM_droite);
  return(CM_droite);

}

void verifygauchedroite() {

    int positionGauche = 180;
    monServo.write(positionGauche);
    delay(300);
    takemesuregauche( );
    distanceGauche = CM_gauche;



    int positionDroite = 0;
    monServo.write(positionDroite);
    delay(300);
    takemesuredroite( );
    distanceDroite = CM_droite;


  }
