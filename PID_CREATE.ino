/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-dc-motor
 */

#include "Encoder.h"

// constants won't change
const int ENA = 12, IN1 = 6, IN2 = 7, ENCA = 2, ENCB = 3; // the Arduino pin connected to the EN1 pin L298N
Encoder myEnc(ENCA, ENCB);
volatile int newPos = 0;
long prevT = 0, userInput, currT, distance, lastPos, totT = 3, startT, prevstartT = 0, abT;
float eprev = 0, tol, deltaT, dedt, eintegral, velocity, v, a = 100;
int speed, oldPos, e, u = 0, vmax = 100; 

//PID constants
float kp = 0.7; // d Tr, i O, d Ts, d SSE lower
float ki = 0.3; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.008; // sd Tr, d O, d Ts, N/A SSE lower

// the setup function runs once when you press reset or power the board
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

// the loop function runs over and over again forever
void loop() {

  if (Serial.available()) {

    userInput = Serial.parseInt();
    Serial.print("target: ");
    Serial.println(userInput);
    distance = abs(oldPos - userInput);
    tol = 0.05*distance;
    newPos = myEnc.read();
    Serial.println(newPos);

    if (newPos <= userInput - tol || newPos >= userInput + tol) {
      while (newPos <= userInput - tol || newPos >= userInput + tol) {
        setMotor(v, u, abT, currT, prevT, totT, startT, prevstartT, distance, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol);
        eprev = e;
        lastPos = newPos;
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

}

void setMotor(float v, int u, long abT, long currT, long prevT, long totT, long startT, long prevstartT, long distance, float eprev, float eintegral, float kp, float kd, float ki, long userInput, volatile int newPos, int speed, int ENA, int IN1, int IN2, float tol) {

  if (userInput - tol > newPos) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
  }

  else if (userInput + tol < newPos) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
  }
    
  startT = micros();
  v = constrain (v, 0, vmax);

  if (distance <= (0.5*totT*vmax)) {

    while (currT <= 0.5*totT/ (1.0e6)) {

      currT = micros() - startT;  // this is in microseconds
      Serial.print(currT);
      Serial.print("   ");
      v = a*((float)currT / 1.0e6);
      deltaT = (float)(currT - prevT) / 1.0e6;
      prevT = currT;
      velocity = (newPos - lastPos) / deltaT;
      lastPos = newPos;
        
      // error
      e = v - velocity;

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, vmax);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);
    }

    while (currT >= 0.5*totT / (1.0e6) && currT <= totT / (1.0e6)) {
      
      currT = micros() - startT;  // this is in microseconds
      Serial.print(currT);
      Serial.print("   ");
      v = -a*((float)currT / 1.0e6);
      deltaT = (float)(currT - prevT) / 1.0e6;
      prevT = currT;
      velocity = (newPos - lastPos) / deltaT;
      lastPos = newPos;
        
      // error
      e = v - velocity;

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, vmax);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);
      
    }

  }

  else if (distance > (0.5*totT*vmax)) {

    while (v <= vmax) {

      currT = micros() - startT;  // this is in microseconds
      Serial.print(currT);
      Serial.print("   ");
      v = a*((float)currT / 1.0e6);
      deltaT = (float)(currT - prevT) / 1.0e6;
      prevT = currT;
      velocity = (newPos - lastPos) / deltaT;
      lastPos = newPos;
        
      // error
      e = v - velocity;

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, vmax);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

      abT = currT;

    }

    while (currT >= abT && currT < (totT/ (1.0e6)) - deltaT) {
      
      currT = micros() - startT;  // this is in microseconds
      Serial.print(currT);
      Serial.print("   ");
      v = vmax;
      deltaT = (float)(currT - prevT) / 1.0e6;
      prevT = currT;
      velocity = (newPos - lastPos) / deltaT;
      lastPos = newPos;
        
      // error
      e = v - velocity;

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, vmax);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    while (currT >= (totT/ (1.0e6)) - deltaT && currT <= 3/ (1.0e6)) {

      currT = micros() - startT;  // this is in microseconds
      Serial.print(currT);
      Serial.print("   ");
      v = -a*((float)currT / 1.0e6);
      deltaT = (float)(currT - prevT) / 1.0e6;
      prevT = currT;
      velocity = (newPos - lastPos) / deltaT;
      lastPos = newPos;
        
      // error
      e = v - velocity;

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, vmax);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }     

  }

  if (abs(newPos - userInput) < tol){
    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    Serial.println("start spool down");
    while (newPos != myEnc.read()) {

      currT = micros() - startT;
      Serial.print(currT);
      Serial.print("   ");
      deltaT = (float)(currT - prevT) / 1.0e6;
      prevT = currT;
      velocity = (newPos - lastPos) / deltaT;
      lastPos = newPos;
        
      // error
      e = v - velocity;

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, vmax);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }
  }
}
