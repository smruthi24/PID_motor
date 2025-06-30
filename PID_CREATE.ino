/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-dc-motor
 */

#include "Encoder.h"

// constants won't change
const int ENA = 12; // the Arduino pin connected to the EN1 pin L298N
const int IN1 = 6; // the Arduino pin connected to the IN1 pin L298N
const int IN2 = 7; // the Arduino pin connected to the IN2 pin L298N
const int ENCA = 2;
const int ENCB = 3;
Encoder myEnc(ENCA, ENCB);
volatile int newPos = 0;
long prevT = 0;
float eprev = 0;
int speed;
long userInput;
float tol1;
float tol2;
float tol3;
float tol4;
int oldPos;
long currT;
float deltaT;
int e;
float dedt;
float eintegral;
int distance;
int speedprev;
float a;

//PID constants
float kp = 2.0; // d Tr, i O, d Ts, d SSE lower
float ki = 0.003; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.001; // sd Tr, d O, d Ts, N/A SSE lower

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
    distance = oldPos - userInput;

    if (distance <= 500) {
      a = 1.3;
      tol1 = abs(distance)*0.75;
      tol2 = abs(distance)*0.50;
      tol3 = abs(distance)*0.25;
      tol4 = abs(distance)*0.15;
    }
    else if (distance <= 700 && distance > 500) {
      a = 2.0;
      tol1 = abs(distance)*0.60;
      tol2 = abs(distance)*0.30;
      tol3 = abs(distance)*0.15;
    }
    else {
      a = 3.2;
      tol1 = abs(distance)*0.25;
      tol2 = abs(distance)*0.10;
    }
    
    newPos = myEnc.read();
    //noInterrupts();
    Serial.println(newPos);

    if (newPos <= userInput - tol2 || newPos >= userInput + tol2) {
      while (newPos <= userInput - tol2 || newPos >= userInput + tol2) {
        Serial.println("fine tuning");
        setMotor(prevT, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol2);
        eprev = e;
        oldPos = userInput;
        newPos = myEnc.read();
        
      }
      digitalWrite(IN1, HIGH); // control motor A stops
      digitalWrite(IN2, HIGH);  // control motor A stops
    }

    if (newPos <= userInput - tol3 || newPos >= userInput + tol3) {
      while (newPos <= userInput - tol3 || newPos >= userInput + tol3) {
        Serial.println("fine tuning");
        setMotor(prevT, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol3);
        eprev = e;
        oldPos = userInput;
        newPos = myEnc.read();

        }
      digitalWrite(IN1, HIGH); // control motor A stops
      digitalWrite(IN2, HIGH);  // control motor A stops
    }

    if (newPos <= userInput - tol4 || newPos >= userInput + tol4) {
      while (newPos <= userInput - tol4 || newPos >= userInput + tol4) {
        Serial.println("fine tuning");
        setMotor(prevT, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol4);
        eprev = e;
        oldPos = userInput;
        newPos = myEnc.read();

        }
      digitalWrite(IN1, HIGH); // control motor A stops
      digitalWrite(IN2, HIGH);  // control motor A stops
    }

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

void setMotor(long prevT, float eprev, float eintegral, float kp, float kd, float ki, long userInput, volatile int newPos, int speed, int ENA, int IN1, int IN2, float tol){

  if (userInput > newPos) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
        
    while (userInput - tol > newPos) {

      currT = micros();
      deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u + speedprev + a, 0, 100);

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

      speedprev = speed;

    }
        
    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    currT = micros();
    deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
        
    // error
    e = abs(newPos - userInput);

    // derivative
    dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    int u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    speed = constrain(u - speedprev - a, 0, 100);

    analogWrite(ENA, speed); // control the speed

    speedprev = speed;
    
  }    

  else if (userInput < newPos) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
        
    while (userInput + tol < newPos) {

      currT = micros();
      deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u + speedprev + a, 0, 100);

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

      speedprev = speed;

    }

    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    currT = micros();
      deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u - speedprev - a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      speedprev = speed;

  }

  Serial.println("start spool down");
  while (newPos != myEnc.read()) {

    currT = micros();
      deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u - speedprev - a, 0, 100);

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

      speedprev = speed;
  }

}
