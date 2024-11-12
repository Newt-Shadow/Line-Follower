#include <avr/wdt.h>  // Include the watchdog timer library

#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define IR_SENSOR_CENTER 3
#define ZERO_RANGE 5

#define MOTOR_SPEED 180
#define NC_SPEED 180
#define TURN_OFFSET 0
#define SHARP_MS 20
#define REV_SPEED 150
#define REV_MS 100

//Right motor = A
#define ENA 6
#define INA1 9
#define INA2 10

//Left motor = B
#define ENB 5
#define INB1 7
#define INB2 8

// int enableRightMotor=5;
// int rightMotorPin1=9;
// int rightMotorPin2=10;

// //Left motor
// int enableLeftMotor=6;
// int leftMotorPin1=7;
// int leftMotorPin2=8;

bool ledState = false;

void setup() {
  //The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  //If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  //For that we need to increase the frequency of analogWrite.
  //Below line is important to change the frequency of PWM signal on pin D5 and D6
  //Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  //This sets frequecny as 7812.5 hz.
  TCCR0B = TCCR0B & B11111000 | B00000001;

  pinMode(LED_BUILTIN, OUTPUT);    // Set the built-in LED pin as an output
  digitalWrite(LED_BUILTIN, LOW);  // Turn off the built-in LED

  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_CENTER, INPUT);
  rotateMotor(0, 0);
  digitalWrite(LED_BUILTIN, LOW);  // Turn off the built-in LED

  // // Check if the last reset was caused by the watchdog
  if (MCUSR & (1 << WDRF)) {
    MCUSR &= ~(1 << WDRF);            // Clear the watchdog reset flag
    digitalWrite(LED_BUILTIN, HIGH);  // Turn off the built-in LED
  }

  // Watchdog timer, incase arduino is stuck
  wdt_enable(WDTO_250MS);
}

int kp = 150;
int kd = 1;
int ki = 0.01;

// NOT PARAMS
int rightIRSensorValue = 0;
int leftIRSensorValue = 0;
int centerIRSensorValue = 0;

// BLACK = HIGH AND WHITE = LOW
float prev = 0;
float integral = 0;
bool rev = false;

float error = 0;

void loop() {
  rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
  centerIRSensorValue = digitalRead(IR_SENSOR_CENTER);

  error = leftIRSensorValue - rightIRSensorValue;

  if(rightIRSensorValue == LOW && leftIRSensorValue == LOW && centerIRSensorValue == LOW && prev == 0 && error == 0) {
    rotateMotorSharp(REV_SPEED, -REV_SPEED, REV_MS);
    return;
  }
  else if (prev != 0 && rightIRSensorValue == LOW && leftIRSensorValue == LOW && centerIRSensorValue == LOW) {
      error = prev;
  }

  integral += error;
  float P = kp * error;
  float I = ki * integral;
  float D = kd * (prev - error);
  float correction = P + I + D;

  prev = error;

  //If center sensors detects black line, then go straight
  if (correction < ZERO_RANGE && correction > -ZERO_RANGE) {
    if (centerIRSensorValue == HIGH)
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    else
      rotateMotor(NC_SPEED, NC_SPEED);
  } else if (rightIRSensorValue == LOW && leftIRSensorValue == LOW && centerIRSensorValue == LOW) {
    if (prev != 0)
      rotateMotorSharp(correction + TURN_OFFSET, -correction - TURN_OFFSET, SHARP_MS);
  } else
    rotateMotor(correction + TURN_OFFSET, -correction - TURN_OFFSET);
  // Reset watchdog timer
  wdt_reset();
}

void rotateMotorSharp(int rightMotorSpeed, int leftMotorSpeed, int delayms) {
  if (rightMotorSpeed < 0) {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
  } else {
    digitalWrite(INA1, HIGH);
    digitalWrite(INA2, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, HIGH);
  } else {
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
  }

  analogWrite(ENA, abs(rightMotorSpeed));
  analogWrite(ENB, abs(leftMotorSpeed));
  delay(delayms);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
  } else {
    digitalWrite(INA1, HIGH);
    digitalWrite(INA2, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, HIGH);
  } else {
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
  }

  analogWrite(ENA, abs(rightMotorSpeed));
  analogWrite(ENB, abs(leftMotorSpeed));
}