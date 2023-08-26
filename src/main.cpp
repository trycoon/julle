/**
 * @file main.cpp
 * @author Henrik Östman (trycoon@gmail.com)
 * @version 0.1
 * @date 2023-08-21
 */

#include <Arduino.h>

#define FORWARD HIGH
#define BACKWARD LOW
#define SPEED_ADJUST_DELAY 30  // how many milliseconds we should wait before adjusting the speed. Lower value gives a more responsive throttle, higher value gives a smoother ride.
#define MAX_FORWARD_SPEED 100  // in percent
#define MAX_BACKWARD_SPEED 30  // in percent

const uint8_t ledPin = 13;                  // Onboard LED connected to digital pin 13
const uint8_t motorPin = 3;                 // Motor PWM pin, connected to a non-inverted OP-AMP
const uint8_t throttePin = A0;
const uint8_t testFullSpeedPin = 4;        // Pull LOW to run motor at 100%, pull HIGH to go back to ordinary speed.
const uint8_t testStopPin = 5;             // Pull LOW to run motor at 0%, pull HIGH to go back to ordinary speed.
const uint8_t emergencyBrakePin = 6;       // Pull LOW to stop motor without slowing down, this also stops the loop, so you have to power-cycle the Arduino to be able to run it again!
const uint8_t forwardReverseButtonPin = 7; // Pull LOW to to run backward, pull HIGH to run forward.
//const uint8_t maxSpeedSelectorPin = 8;     // Pull LOW to to run at half MAX_FORWARD_SPEED, pull HIGH to run at full MAX_FORWARD_SPEED.
const uint8_t motorDirectionPin = 9;        // Controls the direction of the motor, set to LOW to to run backward, set to HIGH to run forward.

uint8_t current_speed = 0;                  // current speed the motor should run at, 0-100 (%)
uint8_t target_speed = 0;                   // the speed we aiming the current_speed should have eventually, 0-100 (%)
uint8_t direction = FORWARD;                // driving direction of car
unsigned long lastSpeedAdjustTime = 0;  

// declare functions:
void setMotorSpeed(int speed);
void setDirection();
void updateCurrentSpeed();
void adjustMotorByThrottle();

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);

  pinMode(throttePin, INPUT);
  pinMode(testFullSpeedPin, INPUT_PULLUP);
  pinMode(testStopPin, INPUT_PULLUP);
  pinMode(emergencyBrakePin, INPUT_PULLUP);
  pinMode(forwardReverseButtonPin, INPUT_PULLUP);
  //pinMode(maxSpeedSelectorPin, INPUT_PULLUP);

  analogWrite(motorPin, current_speed);
  digitalWrite(motorDirectionPin, direction);

  Serial.begin(115200);
  Serial.println("Julle firmware. https://github.com/trycoon/julle");

  // two blinks with LED to show that its working
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);  
}

void loop() {
  if (digitalRead(emergencyBrakePin) == LOW) {
    setMotorSpeed(0);
    Serial.println("EMERGENCY STOP!");
    delay(100);
    exit(0);
  }

  if (digitalRead(testFullSpeedPin) == LOW) {
    target_speed = 100;
  }
  if (digitalRead(testStopPin) == LOW) {
    target_speed = 0;
  }

  if (digitalRead(testFullSpeedPin) == HIGH && digitalRead(testStopPin) == HIGH) {
    // control motor by throttle, unless overridden by "testFullSpeed" or "testStop"-pins.
    adjustMotorByThrottle();
  }

  setDirection();
  updateCurrentSpeed();
  setMotorSpeed(current_speed);
}

void updateCurrentSpeed() {
  unsigned long currentTime = millis(); // overflows every 50-days, hopefully the car is power-cycled before that happens... ;-)
  if (currentTime - lastSpeedAdjustTime > SPEED_ADJUST_DELAY) {
    lastSpeedAdjustTime = currentTime;

    if (current_speed < target_speed && 
      ((direction == FORWARD && current_speed < MAX_FORWARD_SPEED) || (direction == BACKWARD && current_speed < MAX_BACKWARD_SPEED))) {
      current_speed += 1;
    } else if (current_speed > target_speed) {
      current_speed -= 1;
    }
  }
}

void setMotorSpeed(int speed) {
  int realSpeed = map(speed, 0, 100, 0, 255);
  // TODO: will miss logging when reaching target speed, but I'll can live with that.
  if (speed != target_speed) {
    Serial.print("Set speed to: ");
    Serial.print(speed);
    Serial.print("% (");
    Serial.print(realSpeed);
    Serial.print("), traget: ");
    Serial.print(target_speed);
    Serial.println("%");
  }

  analogWrite(ledPin, realSpeed);     // fade LED according to speed
  analogWrite(motorPin, realSpeed);
}

void setDirection() {
  if (current_speed == 0) {
    int newDirection = digitalRead(forwardReverseButtonPin);

    if (direction != newDirection) {
      delay(1500); // allow motor to really stop when changing direction, this saves the sprockets and chains!

      direction = newDirection;

      Serial.print("New direction set: ");
      Serial.println(direction == FORWARD ? "FORWARD" : "BACKWARD");

      digitalWrite(motorDirectionPin, direction);
    }
  }
}

void adjustMotorByThrottle() {
  int throttle = analogRead(throttePin);
  target_speed = map(throttle, 0, 1023, 0, 100);
}