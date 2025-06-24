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
int tol1;
int oldPos;
long currT;
float deltaT;
int e;
float dedt;
float eintegral;
int distance;

// PID constants
float kp = 0.1; // d Tr, i O, d Ts, d SSE lower
float ki = 0.006; // d Tr, i O, i Ts, elim SSE higher
float kd = 0.002; // sd Tr, d O, d Ts, N/A SSE lower

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
    if (distance <= 100) {
      tol1 = abs(distance)*0.25;
    }
    else {
      tol1 = abs(distance)*0.5;
    }
    
    newPos = myEnc.read();
    //noInterrupts();
    Serial.println(newPos);

    setMotor(prevT, eprev, eintegral, kp, kd, ki, userInput, newPos, speed, ENA, IN1, IN2);

    eprev = e;
    oldPos = userInput;
  }
      
    /*if (userInput == 1) {
      digitalWrite(IN1_PIN, HIGH); // control motor A spins clockwise
      digitalWrite(IN2_PIN, LOW);  // control motor A spins clockwise

      for (int speed = 0; speed <= 255; speed++) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
        long newPos = myEnc.read();
        Serial.println(newPos);
      }

      for (int speed = 255; speed >= 0; speed--) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
        long newPos = myEnc.read();
        Serial.println(newPos);
      }
    }

    if (userInput == 2) {
      digitalWrite(IN1_PIN, LOW);   // control motor A spins anti-clockwise
      digitalWrite(IN2_PIN, HIGH);  // control motor A spins anti-clockwise

      for (int speed = 0; speed <= 255; speed++) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
        long newPos = myEnc.read();
        Serial.println(newPos);
      }

      for (int speed = 255; speed >= 0; speed--) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
        long newPos = myEnc.read();
        Serial.println(newPos);
      }
    }*/
}

void setMotor(long prevT, float eprev, float eintegral, float kp, float kd, float ki, long userInput, volatile int newPos, int speed, int ENA, int IN1, int IN2){

  if (userInput - tol1 > newPos) {
    digitalWrite(IN1, HIGH); // control motor A spins clockwise
    digitalWrite(IN2, LOW);  // control motor A spins clockwise
        
    while (userInput - tol1 > newPos) {

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
      speed = constrain(u, 0, 255);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print(" ");
      Serial.print(kp*e);
      Serial.print(" ");
      Serial.print(ki*eintegral);
      Serial.print(" ");
      Serial.println(kd*dedt);

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
    speed = constrain(u, 0, 255);

    analogWrite(ENA, speed); // control the speed
    
  }    

  else if (userInput + tol1 < newPos) {  
    digitalWrite(IN1, LOW); // control motor A spins counterclockwise
    digitalWrite(IN2, HIGH);  // control motor A spins counterclockwise
        
    while (userInput + tol1 < newPos) {

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
      speed = constrain(u, 0, 255);

      analogWrite(ENA, speed); // control the speed

      newPos = myEnc.read();
      delay(10);
      Serial.print(newPos);
      Serial.print(" ");
      Serial.print(kp*e);
      Serial.print(" ");
      Serial.print(ki*eintegral);
      Serial.print(" ");
      Serial.println(kd*dedt);

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
    speed = constrain(u, 0, 255);

    analogWrite(ENA, speed); // control the speed
  }

  Serial.println("start spool down");
  while (newPos != myEnc.read()) {

    currT = micros();
    deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
        
    // error
    e = abs(newPos - userInput);

    // derivative
    dedt = (-e+eprev)/(deltaT);

    // integral
    eintegral = eintegral - e*deltaT;

    int v = -kp*e - kd*dedt - ki*eintegral;

    // motor power
    speed = constrain(v, 0, 255);

    analogWrite(ENA, speed); // control the speed
    
    newPos = myEnc.read();
    delay(10);
    Serial.print(newPos);
      Serial.print(" ");
      Serial.print(kp*e);
      Serial.print(" ");
      Serial.print(ki*eintegral);
      Serial.print(" ");
      Serial.println(kd*dedt);
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
