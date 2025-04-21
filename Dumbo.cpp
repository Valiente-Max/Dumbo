#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

// Pin definitions for ultrasonic sensor (adjust as needed)
#define TRIG_PIN A4
#define ECHO_PIN A5
#define MAX_DISTANCE 200

#define MAX_SPEED 190          // Max speed for motors
#define MAX_SPEED_OFFSET 90    // Offset for motor speed balancing
#define COLL_DIST 10           // Distance at which robot stops/reverses (cm)

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

AF_DCMotor motor1(1, MOTOR12_1KHZ); // Left motor
AF_DCMotor motor2(2, MOTOR12_1KHZ); // Right motor
AF_DCMotor motor3(3, MOTOR12_1KHZ);
AF_DCMotor motor4(4, MOTOR12_1KHZ);

Servo myservo; // Servo for sensor sweep

int leftDistance, rightDistance;
int curDist = 0;
int speedSet = 0;

//-------------------------------------------- SETUP --------------------------------------------------------
void setup() {
  myservo.attach(9);      // Servo on pin 9
  myservo.write(90);      // Center servo (forward)
  delay(1000);
}

//--------------------------------------------- MAIN LOOP ---------------------------------------------------
void loop() {
  myservo.write(90);      // Look forward
  delay(90);
  curDist = readPing();   // Read distance ahead

  if (curDist < COLL_DIST) {
    changePath();         // If obstacle ahead, decide new path
  }
  moveForward();
  delay(500);
}

//--------------------------------------------- PATH DECISION -----------------------------------------------
void changePath() {
  moveStop();
  myservo.write(36);      // Look right
  delay(500);
  rightDistance = readPing();

  delay(500);
  myservo.write(144);     // Look left
  delay(700);
  leftDistance = readPing();

  delay(500);
  myservo.write(90);      // Center servo
  delay(100);

  compareDistance();
}

void compareDistance() {
  if (leftDistance > rightDistance) {
    turnLeft();
  } else if (rightDistance > leftDistance) {
    turnRight();
  } else {
    turnAround();
  }
}

//--------------------------------------------- SENSOR READING ----------------------------------------------
int readPing() {
  delay(70);
  unsigned int uS = sonar.ping();
  int cm = uS / US_ROUNDTRIP_CM;
  return cm;
}

//--------------------------------------------- MOTOR CONTROL -----------------------------------------------
void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    motor1.setSpeed(speedSet + MAX_SPEED_OFFSET);
    motor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
    motor3.setSpeed(speedSet + MAX_SPEED_OFFSET);
    motor4.setSpeed(speedSet + MAX_SPEED_OFFSET);
    delay(5);
  }
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    motor1.setSpeed(speedSet + MAX_SPEED_OFFSET);
    motor2.setSpeed(speedSet + MAX_SPEED_OFFSET);
    motor3.setSpeed(speedSet + MAX_SPEED_OFFSET);
    motor4.setSpeed(speedSet + MAX_SPEED_OFFSET);

    delay(5);
  }
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  delay(400);
  moveForward();
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(400);
  moveForward();
}

void turnAround() {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);


  delay(800);
    moveForward();}