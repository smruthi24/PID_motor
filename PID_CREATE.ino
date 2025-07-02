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
float tol;
int oldPos;
long currT;
float deltaT;
int e;
float dedt;
float eintegral;
int distance;
float v;
int u = 0;
long totT;
long startT;
long prevstartT = 0;
long abT;


//PID constants
float kp = 2.0; // d Tr, i O, d Ts, d SSE lower
float ki = 0.3; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.1; // sd Tr, d O, d Ts, N/A SSE lower

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
    tol = 0.1*distance;
    newPos = myEnc.read();
    Serial.println(newPos);

    if (newPos <= userInput - tol || newPos >= userInput + tol) {
      while (newPos <= userInput - tol || newPos >= userInput + tol) {
        setMotor(v, u, abT, currT, prevT, totT, startT, prevstartT, distance, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol);
        eprev = e;
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

void setMotor(float v, int u, long abT, long currT, long prevT, long totT, long startT, long prevstartT, int distance, float eprev, float eintegral, float kp, float kd, float ki, long userInput, volatile int newPos, int speed, int ENA, int IN1, int IN2, float tol) {

  if (userInput - tol > newPos) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
  }

  else if (userInput + tol < newPos) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
  }
    
  currT = (micros())/(1.0e6);

  if (distance <= 300) {

    while (currT <= 1.5) {

      v = 3.7*currT;
      currT = (micros())/(1.0e6);
      Serial.print(currT);
      Serial.print("   ");
      deltaT = ((float) (currT - prevT));
      prevT = currT;
        
      // error
      e = abs(v - u);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, 100);

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

    while (currT >= 1.5 && currT <= 3) {
      
      v = -3.7*currT;
      currT = (micros())/(1.0e6);
      Serial.print(currT);
      Serial.print("   ");
      deltaT = ((float) (currT - prevT));
      prevT = currT;
        
      // error
      e = abs(v - u);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, 100);

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

  else if (distance > 300) {

    startT = micros() - prevstartT;
    prevstartT = startT;

    while (v <= 100) {

      currT = ((micros())/(1.0e6)) - startT;
      Serial.print(currT);
      Serial.print("   ");
      v = 3.7*currT;
      deltaT = ((float) (currT - prevT));
      prevT = currT;
        
      // error
      e = abs(v - u);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, 100);

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

    v = constrain (v, 0, 100);

    abT = currT;

    currT = ((micros())/(1.0e6)) - startT;

    while (currT >= abT && currT < 3 - deltaT) {
      
      currT = ((micros())/(1.0e6)) - startT;
      Serial.print(currT);
      Serial.print("   ");
      v = 100;
      deltaT = ((float) (currT - prevT));
      prevT = currT;
        
      // error
      e = abs(v - u);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, 100);

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

    while (currT >= 3 - deltaT && currT <= 3) {

      currT = ((micros())/(1.0e6)) - startT;
      Serial.print(currT);
      Serial.print("   ");
      v = -3.7*currT;
      deltaT = ((float) (currT - prevT));
      prevT = currT;
        
      // error
      e = abs(v - u);

      // derivative
      dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      u = kp*e + kd*dedt + ki*eintegral;

      // motor power
      speed = constrain(u, 0, 100);

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

  digitalWrite(IN1, HIGH); // control motor A stops
  digitalWrite(IN2, HIGH);  // control motor A stops

  Serial.println("start spool down");
  while (newPos != myEnc.read()) {

    currT = ((micros())/(1.0e6));
    Serial.print(currT);
    Serial.print("   ");
    deltaT = ((float) (currT - prevT));
    prevT = currT;
      
    // error
    e = abs(v - u);

    // derivative
    dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    speed = constrain(u, 0, 100);

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
