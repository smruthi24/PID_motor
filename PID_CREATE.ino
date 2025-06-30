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

long currT = millis();
float tol = 0.05;
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
    Serial.println(newPos);

    if (newPos <= userInput - tol || newPos >= userInput + tol) {
      while (newPos <= userInput - tol || newPos >= userInput + tol) {
        //Serial.println("fine tuning");
        setMotor(prevT, totT, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2, tol);
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

void setMotor(long prevT, long totT, float eprev, float eintegral, float kp, float kd, float ki, long userInput, volatile int newPos, int speed, int ENA, int IN1, int IN2, float tol){

  if (userInput > newPos) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
        
    while (currT <= 0.33*totT) {

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
      speed = constrain(u + a, 0, 100);

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

    while (currT <= 0.66*totT && currT >= 0.33*totT) {

      currT = micros();
      //deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

      // motor power
      speed = constrain(a, 0, 100);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print("    speed: ");
      Serial.println(speed);

    }

    while (currT <= totT && currT >= 0.66*totT) {

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

  else if (userInput < newPos) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
        
    while (currT <= 0.33*totT) {

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
      speed = constrain(u + a, 0, 100);

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

    while (currT <= 0.66*totT && currT >= 0.33*totT) {

      currT = micros();
      //deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

      // motor power
      speed = constrain(a, 0, 100);

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

    while (currT <= totT && currT >= 0.66*totT) {

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
