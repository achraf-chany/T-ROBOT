#include <Servo.h>
float distance, distanceDroite, distanceGauche;

// Définition des broches pour les capteurs IR
const int IR_Left_Sensor = 5;  // Capteur gauche
const int IR_Right_Sensor = 6; // Capteur droit

// Définition des broches pour les moteurs
#define motor_left_pwm 10       // PWM pour le moteur gauche
#define control_motor_left_1 12 // Contrôle du sens du moteur gauche
#define control_motor_left_2 13 // Contrôle du sens du moteur gauche
#define motor_right_pwm 9       // PWM pour le moteur droit
#define control_motor_right_1 7 // Contrôle du sens du moteur droit
#define control_motor_right_2 8 // Contrôle du sens du moteur droit

// Définition des vitesses des moteurs
#define speedMax    255
#define speedMaxleft   250
#define speedMinright    150
#define speedMinleft    145
#define speedRight  200
#define speedLeft   195
#define BaseSpeed   150
#define noSpeed     0

// Définition des broches pour le capteur ultrason
const int TrigPin = 4;       // Broche TRIG
const int EchoPin = 2;       // Broche ECHO
const int servoPin = 11;     // Broche du servomoteur

// Initialisation du servomoteur
Servo myServo;

// Définition des angles du servomoteur
const int angleCentre = 28;  // Angle central
const int angleGauche = 180; // Angle pour regarder à gauche
const int angleDroite = 0;   // Angle pour regarder à droite

// Distance seuil pour détecter un obstacle (en cm)
const float distanceSeuil = 9.0;

void setup() {
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  myServo.attach(servoPin);
  myServo.write(angleCentre);
  pinMode(IR_Left_Sensor, INPUT);
  pinMode(IR_Right_Sensor, INPUT);
  pinMode(motor_left_pwm, OUTPUT);
  pinMode(control_motor_left_1, OUTPUT);
  pinMode(control_motor_left_2, OUTPUT);
  pinMode(motor_right_pwm, OUTPUT);
  pinMode(control_motor_right_1, OUTPUT);
  pinMode(control_motor_right_2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  distance = ObstacleDistance();
  if (distance <= distanceSeuil) {
    eviter_obstacle();
  } else {
    suivreLigne();
  }
}

void eviter_obstacle() {
  arreter();
  delay(500);
  distanceGauche = verifierDistance(angleGauche);
  if (distanceGauche > distanceSeuil) {
    verifierDistance(angleCentre);
    tournerGauche();
    delay(500);
  } else {
    distanceDroite = verifierDistance(angleDroite);
    if (distanceDroite > distanceSeuil) {
      verifierDistance(angleCentre);
      tournerDroite();
      delay(500);
    } else {
      verifierDistance(angleCentre);
      reculer();
      delay(500);
      arreter();
      return;
    }
  }
  arreter();
  delay(200);
  avancer();
  delay(800);
  arreter();
  delay(200);
  tournerGauche();
  delay(370);
  arreter();
  delay(200);
  avancer();
  delay(200);
  arreter();
  delay(200);
  tournerGauche();
  delay(500);
  arreter();
  delay(200);
  chercher_ligne();
}

float verifierDistance(int angle) {
  myServo.write(angle);
  delay(500);
  return ObstacleDistance();
}

float ObstacleDistance() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  long duration = pulseIn(EchoPin, HIGH, 10000);
  if (duration == 0) {
    return -1.0;
  }
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

void chercher_ligne() {
  int timeout = 5000;
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    int leftSensor = digitalRead(IR_Left_Sensor);
    int rightSensor = digitalRead(IR_Right_Sensor);
    if (leftSensor == 0 || rightSensor == 0) {
      arreter();
      suivreLigne();
      return;
    }
    avancer();
    delay(300);
    arreter();
    leftSensor = digitalRead(IR_Left_Sensor);
    rightSensor = digitalRead(IR_Right_Sensor);
    if (leftSensor == 0 || rightSensor == 0) {
      arreter();
      suivreLigne();
      return;
    }
  }
  arreter();
  Serial.println("Erreur : Ligne non détectée après recherche.");
}

void avancer() {
  digitalWrite(control_motor_left_1, LOW);
  digitalWrite(control_motor_left_2, HIGH);
  analogWrite(motor_left_pwm, speedMaxleft);
  digitalWrite(control_motor_right_1, LOW);
  digitalWrite(control_motor_right_2, HIGH);
  analogWrite(motor_right_pwm, speedMax);
}

void tournerGauche() {
  digitalWrite(control_motor_left_1, HIGH);
  digitalWrite(control_motor_left_2, LOW);
  analogWrite(motor_left_pwm, speedMinleft);
  digitalWrite(control_motor_right_1, LOW);
  digitalWrite(control_motor_right_2, HIGH);
  analogWrite(motor_right_pwm, speedRight);
}

void tournerDroite() {
  digitalWrite(control_motor_left_1, LOW);
  digitalWrite(control_motor_left_2, HIGH);
  analogWrite(motor_left_pwm, speedLeft);
  digitalWrite(control_motor_right_1, HIGH);
  digitalWrite(control_motor_right_2, LOW);
  analogWrite(motor_right_pwm, speedMinright);
}

void reculer() {
  digitalWrite(control_motor_left_1, HIGH);
  digitalWrite(control_motor_left_2, LOW);
  analogWrite(motor_left_pwm, speedLeft);
  digitalWrite(control_motor_right_1, HIGH);
  digitalWrite(control_motor_right_2, LOW);
  analogWrite(motor_right_pwm, speedRight);
}

void arreter() {
  digitalWrite(control_motor_left_1, LOW);
  digitalWrite(control_motor_left_2, LOW);
  analogWrite(motor_left_pwm, noSpeed);
  digitalWrite(control_motor_right_1, LOW);
  digitalWrite(control_motor_right_2, LOW);
  analogWrite(motor_right_pwm, noSpeed);
}

void suivreLigne() {
  int leftSensor = digitalRead(IR_Left_Sensor);
  int rightSensor = digitalRead(IR_Right_Sensor);
  if (leftSensor == 0 && rightSensor == 0) {
    avancer();
  } else if (leftSensor == 1 && rightSensor == 0) {
    tournerGauche();
  } else if (leftSensor == 0 && rightSensor == 1) {
    tournerDroite();
  } else if (leftSensor == 1 && rightSensor == 1) {
    reculer();
  }
}
