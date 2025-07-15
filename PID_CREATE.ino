#include "Encoder.h"
#include "LibPrintf.h"

// constants won't change
const int ENA = 12, IN1 = 6, IN2 = 7, ENCA = 2, ENCB = 3, INP = 9; // the Arduino pin connected to the EN1 pin L298N
volatile int newPos = 0;
long userInput;
float speed, oldPos, er, eri, u = 0, a = 4000, vmax = 6000, x, setpoint, speedprev;
float tolm, tolp, delT = 10, ederiv, einteg, ta, tb, t, startT, totT, distance;

Encoder myEnc(ENCA, ENCB);

//PID constants
float kp = 1.0; // d Tr, i O, d Ts, d SSE lower
float ki = 0.01; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.1; // sd Tr, d O, d Ts, N/A SSE lower

void setup() {
  // initialize digital pins as outputs.
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(INP, INPUT_PULLUP);

  Serial.begin(230400);
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

    tolm = userInput - 15;
    tolp = userInput + 15;

    startT = micros();

    while (newPos <= tolm || newPos >= tolp) {
      vProf(distance);
      oldPos = userInput;
      setpoint = 0;
    }


    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    delay(1000);
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

void motorMove(float setpoint) {

  newPos = (float) myEnc.read();

  if (setpoint - newPos > 15) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
  }

  else if (setpoint - newPos < 15) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise

  }

  else {
    digitalWrite(IN1, LOW); // control motor A stops
    digitalWrite(IN2, LOW);  // control motor A stops

  }

}

void vProf(float distance) {

  ta = (float)vmax / a;
  totT = (abs(distance)*a + vmax*vmax) / (a*vmax);
  tb = totT - 2*ta;
  t = (float)(micros() - startT)/(1.0e6);

  while ((newPos <= tolm || newPos >= tolp) && t < 10) { 

    if (t < ta) {
      x = (0.5*a*t*t);
    }

    else if (t >= ta && t < ta + tb) {
      x = (0.5*a*ta*ta + vmax*(t - ta));
    }

    else {
      x = (0.5*a*ta*ta + vmax*tb + vmax*(t - ta - tb) - 0.5*a*(t - ta - tb)*(t - ta - tb));
    }

    t = (float)(micros() - startT)/(1.0e6);

    //printf(" total time: %f ramp time: %f cruise time: %f \n", totT, ta, tb);
    int pullup = digitalRead(INP);

    if (pullup == LOW) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    }
    
    newPos = myEnc.read();
    setpoint = userInput > oldPos ? oldPos + x : oldPos - x;
    printf(" time: %f setpoint: %f ", t, setpoint);
    delay(10);
    PIDcalc(setpoint);
  }
  
  digitalWrite(IN1, LOW); // control motor A stops
  digitalWrite(IN2, LOW);  // control motor A stops

}

void PIDcalc(float setpoint) {

  newPos = myEnc.read();
  er = abs(setpoint - newPos);
  ederiv = (er - eri) / delT;
  einteg = einteg + er * delT;

  u = kp*er + ki*einteg + kd*ederiv;

  if (t < ta) {
    speed = constrain(u, 30, 255);
  }
  else {
    speed = constrain(u, 0, 255);
  }
  
  analogWrite(ENA, speed);
  motorMove(setpoint);

  Serial.print(" position: ");
  newPos = myEnc.read();
  Serial.print(newPos);
  Serial.print(" kp*e: ");
  Serial.print(kp*er);
  Serial.print(" ki*e: ");
  Serial.print(ki*einteg);
  Serial.print(" kd*e: ");
  Serial.print(kd*ederiv);
  Serial.print(" speed: ");
  Serial.println(speed);

  eri = er;
  er = 0;
  ederiv = 0;
  einteg = 0;

}
