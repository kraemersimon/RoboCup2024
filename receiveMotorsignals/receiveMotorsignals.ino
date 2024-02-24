#define MOTOR_A_EN 3
#define MOTOR_A_IN1 4
#define MOTOR_A_IN2 2
#define MOTOR_B_EN 6
#define MOTOR_B_IN1 7
#define MOTOR_B_IN2 8

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
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

    if (motor == 0) { // Left Motor
      if (speed == 0) { // Stop
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
        Serial.println("Motor A stop");
      } else {
        if (direction == 0) { // Reverse
          digitalWrite(MOTOR_A_IN1, HIGH);
          digitalWrite(MOTOR_A_IN2, LOW);
          Serial.println("Motor A reverse");
        } else { // Forward
          digitalWrite(MOTOR_A_IN1, LOW);
          digitalWrite(MOTOR_A_IN2, HIGH);
          Serial.println("Motor A forward");
        }
      }
      analogWrite(MOTOR_A_EN, speed); // Send speed to Left Motor
    } else { // Right Motor
      if (speed == 0) { // Stop
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
        Serial.println("Motor B stop");
      } else {
        if (direction == 0) { // Reverse
          digitalWrite(MOTOR_B_IN1, HIGH);
          digitalWrite(MOTOR_B_IN2, LOW);
          Serial.println("Motor B reverse");
        } else { // Forward
          digitalWrite(MOTOR_B_IN1, LOW);
          digitalWrite(MOTOR_B_IN2, HIGH);
          Serial.println("Motor B forward");
        }
      }
      analogWrite(MOTOR_B_EN, speed); // Send speed to Right Motor
    }
  }
}
