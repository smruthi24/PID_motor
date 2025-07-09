#include "Encoder.h"
// set delT to 10 set totT equation make function for x position and for PID constants
// constants won't change
const int ENA = 12, IN1 = 6, IN2 = 7, ENCA = 2, ENCB = 3; // the Arduino pin connected to the EN1 pin L298N
volatile int newPos = 0;
long ti = 0, userInput, t, distance, startT, totT;
int speed, oldPos, er, eri, u = 0, a = 1000, vmax = 100, xa, xb, xc, x;
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
    oldPos = myEnc.read();
    Serial.print(" current position: ");
    Serial.println(oldPos);

    distance = oldPos - userInput;
    tolm = userInput - abs(distance) * 0.05;
    tolp = userInput + abs(distance) * 0.05;
    ta = (float)vmax / a;

    if (abs(distance) < a * ta * ta) {
      ta = sqrt(abs(distance) / a);
      totT = 2 * ta;
      tb = 0;
    }

    else {
      totT = (abs(distance)*a + vmax*vmax) / (a*vmax);
      tb = totT - 2*ta;
    }
    
    startT = micros();
    motorMove(distance);

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

void motorMove(long distance) {

  bool direction = (distance > 0);
  digitalWrite(IN1, direction ? HIGH:LOW);
  digitalWrite(IN2, direction ? LOW:HIGH);

  t = (float)(micros() - startT)/(1.0e6);

  while (t < totT) {
    t = (float)(micros() - startT)/(1.0e6);
    Serial.print("time: ");
    Serial.println(t);

    if (t < ta) {
      x = 0.5*a*t*t;
    }

    else if (/*t >= ta &&*/ t < ta + tb) {
      x = 0.5*a*ta*ta + vmax*(t - ta);
    }

    else /*if (t >= ta + tb && t <= totT)*/ {
      x = 0.5*a*ta*ta + vmax*tb + vmax*(t - ta - tb) - 0.5 * a * (t - ta - tb) * (t - ta - tb);
    }

    if (!direction) x = -x;
    int setpoint = myEnc.read() + x;
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
  speed = constrain(u, 0, 255);
  analogWrite(ENA, speed);

  newPos = myEnc.read();
  /*Serial.print(" position: ");
  Serial.print(newPos);
  Serial.print(" kp*e: ");
  Serial.print(kp*er);
  Serial.print(" ki*e: ");
  Serial.print(ki*einteg);
  Serial.print(" kd*e: ");
  Serial.print(kd*ederiv);
  Serial.print(" speed: ");
  Serial.println(speed);*/

}

