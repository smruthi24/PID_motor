#include "Encoder.h"
// set delT to 10 set totT equation make function for x position and for PID constants
// constants won't change
const int ENA = 12, IN1 = 6, IN2 = 7, ENCA = 2, ENCB = 3; // the Arduino pin connected to the EN1 pin L298N
volatile int newPos = 0;
long ti = 0, userInput, t, distance, startT, totT;
int speed, oldPos, er, eri, u = 0, a = 10, vmax = 100, xa, xb, xc, x;
float tolm, tolp, delT = 10/(1.0e6), ederiv, einteg, ta, tb;

Encoder myEnc(ENCA, ENCB);

//PID constants
float kp = 0.6; // d Tr, i O, d Ts, d SSE lower
float ki = 0.03; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.002; // sd Tr, d O, d Ts, N/A SSE lower

void setup() {
  // initialize digital pins as outputs.
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  Serial.begin(9600);
  Serial.println("enter number of counts");
  
  while(!Serial);  
}

void loop () {

  if (Serial.available()) {
    userInput = Serial.parseInt();
    Serial.print("target: ");
    Serial.print(userInput);
    newPos = myEnc.read();
    Serial.println(newPos);

    distance = oldPos - userInput;
    tolm = userInput - abs(distance) * 0.05;
    tolp = userInput + abs(distance) * 0.05;
    totT = (distance*a + vmax*vmax) / (a*vmax);

    while (newPos <= tolm|| newPos >= tolp) {
      startT = micros();
      motorMove(distance);
      vProf(distance);
    }

  digitalWrite(IN1, HIGH); // control motor A stops
  digitalWrite(IN2, HIGH);  // control motor A stops


  }

}

void motorMove(long distance) {
  if (distance > 0) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
  }

  else if (distance < 0) {
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
  }

  else {
    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops
  }

}

void vProf(long distance) {
  
  if (distance <= (0.5*totT*vmax)) {
    ta = 0.5*totT*vmax;
    t = micros() - startT;
    Serial.print("time: ");
    Serial.println(t);

    while (t < ta) {
      xa = 0.5*a*pow((float)t / 1.0e6, 2); // convert t to seconds
      PIDcalc(xa);
      t = micros() - startT;
    }

    while (t >= ta && t < totT) {
      xb = xa + vmax*(((float)t / (1.0e6))- ta) - 0.5*a*(((float)t / (1.0e6)) - ta)*(((float)t / (1.0e6))- ta); // convert t to seconds
      PIDcalc(xb);
      t = micros() - startT;
    }
  }

  else {
    ta = (float)vmax / a;
    t = micros() - startT;
    Serial.print("time: ");
    Serial.println(t);

    while (t < ta) {
      xa = 0.5*a*pow((float)t / 1.0e6, 2);
      PIDcalc(xa);
      t = micros() - startT;
    }

    while (t >= ta && t < totT - ta) {
      xb = xa + vmax*(((float)t  / (1.0e6)) - ta);
      PIDcalc(xb);
    }

    while (t >= totT - ta && t <= totT) {
      xc = xa + vmax*((totT*(1.0e6)) - (2*((float)ta / (1.0e6)))) - 0.5*a*(((float)t / (1.0e6)) - (totT - ta))*(((float)t / (1.0e6)) - (totT - ta));
      PIDcalc(xc);
    }

  }

}

void PIDcalc(int x) {
  
  newPos = myEnc.read();
  er = (int)(x - newPos);
  ederiv = (er - eri) / delT;
  einteg += er * delT;

  u = kp*er + ki*einteg + kd*ederiv;
  speed = constrain(u, 0, vmax);
  analogWrite(ENA, speed);

  newPos = myEnc.read();
  delay(10);
  Serial.print(" position: ");
  Serial.print(newPos);
  Serial.print(" kp*e: ");
  Serial.print(kp*er);
  Serial.print(" ki*e: ");
  Serial.print(ki*einteg);
  Serial.print(" kd*e: ");
  Serial.print(kd*ederiv);
  Serial.print(" speed: ");
  Serial.println(speed);

}
