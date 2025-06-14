/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-dc-motor
 */
#include "Encoder.h"

// constants won't change
const int ENA_PIN = 12; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 6; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 7; // the Arduino pin connected to the IN2 pin L298N
Encoder myEnc(2, 3);
long newPos = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pins as outputs.
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("enter number of counts");
  
  while(!Serial);  
}


// the loop function runs over and over again forever
void loop() {

  if (Serial.available()) {
    long userInput = Serial.parseInt();

    if (userInput > newPos) {
      for (int i = newPos; i < userInput; i++) {
          digitalWrite(IN1_PIN, HIGH); // control motor A spins clockwise
          digitalWrite(IN2_PIN, LOW);  // control motor A spins clockwise
          analogWrite(ENA_PIN, 60); // control the speed
          myEnc.write(i);
          newPos = myEnc.read();
          Serial.println(newPos);

      }
    }    

    else if (userInput < newPos) {  
      for (int j = newPos; j > userInput; j--) {
            digitalWrite(IN1_PIN, LOW); // control motor A spins counterclockwise
            digitalWrite(IN2_PIN, HIGH);  // control motor A spins counterclockwise
            analogWrite(ENA_PIN, 60); // control the speed
            myEnc.write(j);
            newPos = myEnc.read();
            Serial.println(newPos);
      }
    }

    else if (userInput = newPos) {
      digitalWrite(IN1_PIN, HIGH); // control motor A stops
      digitalWrite(IN2_PIN, HIGH);  // control motor A stops

      }
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



