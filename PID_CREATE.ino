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
int oldPos;
float deltaT;
int e;
float dedt;
float eintegral;
int distance;
int speedprev;
long currT;
long startT;

long time = currT - startT;
float tol = 0.10;
long totT = 5000;
float a = 3.7;
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

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {

    userInput = Serial.parseInt();
    Serial.print("target: ");
    Serial.println(userInput);
    distance = abs(oldPos - userInput);
    newPos = myEnc.read();

    if (newPos <= userInput - tol || newPos >= userInput + tol) {
      while (newPos <= userInput - tol || newPos >= userInput + tol) {
        setMotor(prevT, totT, startT, time, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol);
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

void setMotor(long prevT, long totT, long startT, long time, float eprev, float eintegral, float kp, float kd, float ki, long userInput, volatile int newPos, int speed, int ENA, int IN1, int IN2, float tol){
  if (userInput > newPos) {

    currT = millis();
    startT = currT;
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
        
    while (time <= 0.33*totT - 50) {

      currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
      time = currT - startT;
      deltaT = ((float) (currT - prevT))/( 1.0e3 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u + a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    while (time <= 0.66*totT - 50 && time >= 0.33*totT) {

      currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
      time = currT - startT;
      prevT = currT;

      // motor power
      speed = constrain(a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    while (time <= totT - 50 && time >= 0.66*totT) {

      currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
      time = currT - startT;
      deltaT = ((float) (currT - prevT))/( 1.0e3 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u - a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    "); 
    time = currT - startT;
    prevT = currT;
        
    // error
    e = abs(newPos - userInput);

    // derivative
    dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    int u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    speed = constrain(u - a, 0, 100);

    analogWrite(ENA, speed); // control the speed

    newPos = myEnc.read();
    delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);
    
  }    

  else if (userInput < newPos) {  
    
    currT = millis();
    startT = currT;
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
        
    while (time <= 0.33*totT - 50) {

      currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
      time = currT - startT;
      deltaT = ((float) (currT - prevT))/( 1.0e3 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u + a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    while (time <= 0.66*totT - 50 && time >= 0.33*totT) {

      currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
      time = currT - startT;
      prevT = currT;

      // motor power
      speed = constrain(a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    while (time <= totT - 50 && time >= 0.66*totT) {

      currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
      time = currT - startT;
      deltaT = ((float) (currT - prevT))/( 1.0e3 );
      prevT = currT;
        
      // error
      e = abs(newPos - userInput);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      int u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u - a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    digitalWrite(IN1, HIGH); // control motor A stops
    digitalWrite(IN2, HIGH);  // control motor A stops

    currT = millis();
      Serial.print("current time: ");
      Serial.print(currT);
      Serial.print("    ");
    time = currT - startT;
    deltaT = ((float) (currT - prevT))/( 1.0e3 );
    prevT = currT;
        
    // error
    e = abs(newPos - userInput);

    // derivative
    dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    int u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    speed = constrain(u - a, 0, 100);

    analogWrite(ENA, speed); // control the speed

    newPos = myEnc.read();
    delay(10);
      Serial.print(newPos);
      Serial.print("    time: ");
      Serial.print(time);
      Serial.print("    kp*e: ");
      Serial.print(kp*e);
      Serial.print("    ki*e: ");
      Serial.print(ki*eintegral);
      Serial.print("    kd*e: ");
      Serial.print(kd*dedt);
      Serial.print("    speed: ");
      Serial.println(speed);

  }
  
  /*Serial.println("start spool down");
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
    speed = constrain(u - a, 0, 100);

    analogWrite(ENA, speed); // control the speed

    newPos = myEnc.read();
    Serial.print(newPos);
    Serial.print("    kp*e: ");
    Serial.print(kp*e);
    Serial.print("    ki*e: ");
    Serial.print(ki*eintegral);
    Serial.print("    kd*e: ");
    Serial.print(kd*dedt);
    Serial.print("    speed: ");
    Serial.println(speed);

  }*/

}
