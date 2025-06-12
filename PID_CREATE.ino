/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-dc-motor
 */

// constants won't change
const int ENA_PIN = 12; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 6; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 7; // the Arduino pin connected to the IN2 pin L298N

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pins as outputs.
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("enter 1 for clockwise or 2 for counter clockwise");
  while(!Serial);
}

// the loop function runs over and over again forever
void loop() {

  if (Serial.available()) {
    int userInput = Serial.parseInt();

    if (userInput == 1) {
      digitalWrite(IN1_PIN, HIGH); // control motor A spins clockwise
      digitalWrite(IN2_PIN, LOW);  // control motor A spins clockwise

      for (int speed = 0; speed <= 255; speed++) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
      }

      for (int speed = 255; speed >= 0; speed--) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
      }
    }

    if (userInput == 2) {
      digitalWrite(IN1_PIN, LOW);   // control motor A spins anti-clockwise
      digitalWrite(IN2_PIN, HIGH);  // control motor A spins anti-clockwise

      for (int speed = 0; speed <= 255; speed++) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
      }

      for (int speed = 255; speed >= 0; speed--) {
        analogWrite(ENA_PIN, speed); // control the speed
        delay(20);
      }
    }
  }
}

