#define MOTOR_A_EN 3
#define MOTOR_A_IN1 4
#define MOTOR_A_IN2 2
#define MOTOR_B_EN 6
#define MOTOR_B_IN1 7
#define MOTOR_B_IN2 8 //motors

#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#define PIN A0
#define NUMPIXELS 8
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // stop motors
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_A_EN, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(MOTOR_B_EN, HIGH);

  // set all 8 LEDS to white
  pixels.begin();
  for (int i = 0; i < 8; ++i) pixels.setPixelColor(i, pixels.Color(255, 255, 255));
  pixels.show();
}

void loop() {
  if (Serial.available() >= 1) {
    int receivedByte = Serial.read();
    int motor = (receivedByte >> 7) & 1;
    int direction = (receivedByte >> 6) & 1;
    int speed = receivedByte & 0b00111111;

    // Multiply speed by 4
    speed *= 4;

    // Ensure speed is within the valid range
    speed = constrain(speed, 0, 255);
    // full stop if speed = 0
    if (speed == 0) {
      if (motor == 1) {
          digitalWrite(MOTOR_A_IN1, HIGH);
          digitalWrite(MOTOR_A_IN2, HIGH);
          digitalWrite(MOTOR_A_EN, HIGH);
      } else {
          digitalWrite(MOTOR_B_IN1, HIGH);
          digitalWrite(MOTOR_B_IN2, HIGH);
          digitalWrite(MOTOR_B_EN, HIGH);
      }
    }

    if (motor == 0) { // Left Motor
      if (speed == 0) { // Stop
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
      } else {
        if (direction == 0) { // Reverse
          digitalWrite(MOTOR_A_IN1, HIGH);
          digitalWrite(MOTOR_A_IN2, LOW);
        } else { // Forward
          digitalWrite(MOTOR_A_IN1, LOW);
          digitalWrite(MOTOR_A_IN2, HIGH);;
        }
      }
      analogWrite(MOTOR_A_EN, speed); // Send speed to Left Motor
    } else { // Right Motor
      if (speed == 0) { // Stop
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
      } else {
        if (direction == 0) { // Reverse
          digitalWrite(MOTOR_B_IN1, HIGH);
          digitalWrite(MOTOR_B_IN2, LOW);
        } else { // Forward
          digitalWrite(MOTOR_B_IN1, LOW);
          digitalWrite(MOTOR_B_IN2, HIGH);
        }
      }
      analogWrite(MOTOR_B_EN, speed); // Send speed to Right Motor
    }
  }
}
