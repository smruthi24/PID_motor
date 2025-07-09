#include "Encoder.h"
// set delT to 10 set totT equation make function for x position and for PID constants
// constants won't change
const int ENA = 12, IN1 = 6, IN2 = 7, ENCA = 2, ENCB = 3; // the Arduino pin connected to the EN1 pin L298N
volatile int newPos = 0;
long userInput;
float speed, oldPos, er, eri, u = 0, a = 10000, vmax = 5000, x;
float tolm, tolp, delT = 10/(1.0e6), ederiv, einteg, ta, tb, t, startT, totT, distance;

Encoder myEnc(ENCA, ENCB);

//PID constants
float kp = 0.3; // d Tr, i O, d Ts, d SSE lower
float ki = 0.07; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.08; // sd Tr, d O, d Ts, N/A SSE lower

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
    oldPos = myEnc.read();
    Serial.print(" current position: ");
    Serial.println(oldPos);

    distance = (float)(userInput - oldPos);
    Serial.println(distance);
    tolm = userInput - abs(distance) * 0.05;
    tolp = userInput + abs(distance) * 0.05;
    ta = (float)vmax / a;

    if (abs(distance) < a * ta * ta) {
      ta = sqrt(abs(distance) / a);
      totT = 2 * ta;
      tb = 0;
      Serial.print(" ta: ");
      Serial.print(ta);
      Serial.print(" tb: ");
      Serial.print(tb);
      Serial.print(" totT: ");
      Serial.print(totT);
      Serial.print(" current position: ");
      Serial.print(oldPos);
      Serial.print(" distance: ");      
      Serial.println(distance);
    }

    else {
      totT = (abs(distance)*a + vmax*vmax) / (a*vmax);
      tb = totT - 2*ta;
      Serial.print(" ta: ");
      Serial.print(ta);
      Serial.print(" tb: ");
      Serial.print(tb);
      Serial.print(" totT: ");
      Serial.print(totT);
      Serial.print(" current position: ");
      Serial.print(oldPos);
      Serial.print(" distance: ");
      Serial.println(distance);
    }
    
    startT = micros();
    motorMove(distance);
    vProf(distance);

    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    Serial.println("motor stopped");
    Serial.print("target count: ");
    Serial.print(userInput);
    Serial.print(" actual count: ");
    newPos = myEnc.read();
    Serial.println(newPos);
    Serial.print(kp);
    Serial.print(" ");
    Serial.print(ki);
    Serial.print(" ");
    Serial.println(kd);
    delay(1000);
    Serial.println("enter number of counts");
    userInput = Serial.parseInt();

  }

}

void motorMove(float distance) {

  if (distance > 0) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
    }

  else if (distance < 0) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
  }

  else {
    digitalWrite(IN1, LOW); // control motor A stops
    digitalWrite(IN2, LOW);  // control motor A stops
  }

}

void vProf(float distance) {

  t = (float)(micros() - startT)/(1.0e6);

  while (t < totT) {
    t = (float)(micros() - startT)/(1.0e6);
    Serial.print("time: ");
    Serial.print(t);

    if (t < ta) {
      x = 0.5*a*t*t;
    }

    else if (/*t >= ta &&*/ t < ta + tb) {
      x = 0.5*a*ta*ta + vmax*(t - ta);
    }

    else /*if (t >= ta + tb && t <= totT)*/ {
      x = 0.5*a*ta*ta + vmax*tb + vmax*(t - ta - tb) - 0.5 * a * (t - ta - tb) * (t - ta - tb);
    }

    int setpoint = oldPos + x;
    PIDcalc(setpoint);
    delay(10);

  }

}

void PIDcalc(int setpoint) {
  
  newPos = myEnc.read();
  er = (int)(setpoint - newPos);
  ederiv = (er - eri) / delT;
  einteg += er * delT;

  u = kp*er + ki*einteg + kd*ederiv;
  speed = constrain(u, -100, 100);
  analogWrite(ENA, speed);

  newPos = myEnc.read();
  Serial.print(newPos);
  Serial.print("    kp*e: ");
  Serial.print(kp*er);
  Serial.print("    ki*e: ");
  Serial.print(ki*einteg);
  Serial.print("    kd*e: ");
  Serial.print(kd*ederiv);
  Serial.print("    speed: ");
  Serial.println(speed);

  eri = er;
}
