#ifndef RESCUEROBOTLIB_H
#define RESCUEROBOTLIB_H

#include <Arduino.h>
#include <Servo.h>

// Motor pins for the 4-wheel chassis
#define LEFT_MOTOR_PIN  3
#define RIGHT_MOTOR_PIN 5

// Servo pins for the 4DOF grappler
#define SERVO_BASE_PIN  7
#define SERVO_ARM_PIN   8
#define SERVO_LIFT_PIN  9
#define SERVO_CLAW_PIN  10

// Sensor pins and addresses
#define LINE_SENSOR_PIN A0
#define MAGNETOMETER_ADDRESS 0x0D  // Example I2C address for QMC5883L

class RescueRobot {
public:
  // Constructor and initialization
  RescueRobot();
  void begin(); // Initializes motors, sensors, and servos

  // Navigation functions
  void moveForward(int distance);   // Move forward a specified distance (cm)
  void turn(int angle);             // Turn robot by a given angle (positive for right, negative for left)
  void returnToStart();             // Navigate back to the starting position

  // Mapping and Line following functions
  void updateMap();                 // Update an internal representation of the field
  void displayMap();                // (Optional) Display map data via Serial
  void lineFollow();                // Use line sensor data to follow the board's line

  // Block handling functions
  bool detectBlock();               // Detect if a block is present using the TCRT5000 sensor
  int  identifyBlock();             // Identify the type of block (e.g., red, green, brown)
  void pickBlock();                 // Use the grappler to pick up a block
  void releaseBlock();              // Release or drop the block

  // Grappler individual servo controls
  void setGrapplerBase(int angle);  // Set the base servo angle
  void setGrapplerArm(int angle);   // Set the arm servo angle
  void setGrapplerLift(int angle);  // Set the lift servo angle
  void setGrapplerClaw(int angle);  // Set the claw servo angle

  // Compass and Navigation functions
  void calibrateCompass();          // Calibrate the QMC5883L magnetometer
  int getHeading();                 // Get the current heading in degrees

private:
  void setMotorSpeed(int leftSpeed, int rightSpeed);  // Low-level motor control

  // Private servo objects for the grappler
  Servo grapplerBase;
  Servo grapplerArm;
  Servo grapplerLift;
  Servo grapplerClaw;
};

#endif
