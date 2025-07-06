#include "Encoder.h"

// constants won't change
const int ENA = 12, IN1 = 6, IN2 = 7, ENCA = 2, ENCB = 3; // the Arduino pin connected to the EN1 pin L298N
volatile int newPos = 0;
long ti = 0, userInput, t, distance, startT, ta;
float tol, delT, ederiv, einteg, a = 500, totT = 3;
int speed, oldPos, er, eri, u = 0, vmax = 100, xa, xb, xc;

Encoder myEnc(ENCA, ENCB);

//PID constants
float kp = 0.6; // d Tr, i O, d Ts, d SSE lower
float ki = 0.03; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.02; // sd Tr, d O, d Ts, N/A SSE lower

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

void loop() {

  if (Serial.available()) {

    userInput = Serial.parseInt();
    Serial.print("target: ");
    Serial.println(userInput);
    distance = abs(oldPos - userInput);
    tol = 0.05*distance;
    newPos = myEnc.read();
    Serial.println(newPos);

    while (newPos <= userInput - tol || newPos >= userInput + tol) {

      setMotor();
      oldPos = userInput;
      newPos = myEnc.read();
      Serial.println(newPos);

    }

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

void setMotor() {
  
  if (userInput - tol > newPos) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
  }

  else if (userInput + tol < newPos) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
  }
    
  startT = micros();

  if (distance <= (0.5*totT*vmax)) {
    t = micros() - startT;
    while (t <= (0.5*totT*(1.0e6))) { // convert totT to microseconds
      t = micros() - startT;
      Serial.print("time: ");
      Serial.print(t);
      
      xa = 0.5*a*pow((float)t / 1.0e6, 2); // convert t to seconds
      delT = (float)(t - ti) / (1.0e6); // convert t to seconds
      ti = t;

      newPos = myEnc.read();
      er = (int)(xa - newPos);
      ederiv = (er - eri) / delT;
      einteg += er*delT;

      u = kp*er + ki*einteg + kd*ederiv;
      speed = constrain(u, 0, 100);
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

      ta = t;
      t = micros() - startT;
    }

    while (t > (0.5*totT*(1.0e6) && t <= (totT*(1.0e6)))) { // convert totT to microseconds
      t = micros() - startT;
      Serial.print("time: ");
      Serial.print(t);
      
      xb = xa + vmax*((float)(t - ta) / (1.0e6)) - 0.5*a*pow((float)(t - ta) / (1.0e6), 2); // convert t to seconds
      delT = (float)(t - ti) / (1.0e6); // convert t to seconds
      ti = t;

      newPos = myEnc.read();
      er = abs(xb - newPos);
      ederiv = (er - eri) / delT;
      einteg += er*delT;

      u = kp*er + ki*einteg + kd*ederiv;
      speed = constrain(u, 0, 100);
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

      t = micros() - startT;
    }
  }

  else {
    t = micros() - startT;
    while (u <= vmax) {
      Serial.print(" positive acceleration start ");
      t = micros() - startT;
      Serial.print("time: ");
      Serial.print(t);

      xa = 0.5*a*pow((float)t / 1.0e6, 2);
      delT = (float)(t - ti) / (1.0e6); // convert t to seconds
      ti = t;

      newPos = myEnc.read();
      er = (int)(xa - newPos);
      ederiv = (er - eri) / delT;
      einteg += er*delT;

      u = kp*er + ki*einteg + kd*ederiv;
      speed = constrain(u, 0, 100);
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

      ta = t;
    }

    while (t >= ta && t < (totT*(1.0e6)) - ta ) {
      Serial.print(" constant acceleration start ");
      t = micros() - startT;
      Serial.print("time: ");
      Serial.print(t);

      xb = xa + vmax*((float)(t - ta) / (1.0e6));
      delT = (float)(t - ti) / (1.0e6); // convert t to seconds
      ti = t;

      newPos = myEnc.read();
      er = abs(xb - newPos);
      ederiv = (er - eri) / delT;
      einteg += er*delT;

      u = kp*er + ki*einteg + kd*ederiv;
      speed = constrain(u, 0, 100);
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

      t = micros() - startT;

    }

    while (t >= (totT*(1.0e6)) - ta && t <= totT*(1.0e6)) {
      Serial.print(" negative acceleration start ");
      t = micros() - startT;
      Serial.print("time: ");
      Serial.print(t);

      xc = xa + vmax*((totT*(1.0e6)) - (2*((float)ta / (1.0e6)))) - 0.5*a*pow((float)(t - (totT*(1.0e6) - ta)) / (1.0e6), 2);
      delT = (float)(t - ti) / (1.0e6); // convert t to seconds
      ti = t;

      newPos = myEnc.read();
      er = (int)(xc - newPos);
      ederiv = (er - eri) / delT;
      einteg += er*delT;

      u = kp*er + ki*einteg + kd*ederiv;
      speed = constrain(u, 0, 100);
      analogWrite(ENA, speed);

      if (u > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      } 
      else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      }

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

      t = micros() - startT;

    }
  }
}
