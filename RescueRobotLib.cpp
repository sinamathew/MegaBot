#include "RescueRobotLib.h"
#include <Wire.h>

// Constructor: set default values if needed
RescueRobot::RescueRobot() {
  // Constructor can be empty or initialize variables
}

void RescueRobot::begin() {
  // Initialize motor pins
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  
  // Initialize sensor pin
  pinMode(LINE_SENSOR_PIN, INPUT);
  
  // Initialize I2C for the magnetometer
  Wire.begin();
  
  // Attach grappler servos to their pins
  grapplerBase.attach(SERVO_BASE_PIN);
  grapplerArm.attach(SERVO_ARM_PIN);
  grapplerLift.attach(SERVO_LIFT_PIN);
  grapplerClaw.attach(SERVO_CLAW_PIN);
  
  // Set default positions for servos
  setGrapplerBase(0);
  setGrapplerArm(0);
  setGrapplerLift(90); // Assuming 90 is the neutral position for lift
  setGrapplerClaw(0);  // Open position

  // Calibrate the compass
  calibrateCompass();
}

void RescueRobot::moveForward(int distance) {
  // Calculate travel time based on distance (simple delay-based method)
  int travelTime = distance * 50; // Adjust factor as needed for your chassis
  setMotorSpeed(150, 150);
  delay(travelTime);
  setMotorSpeed(0, 0);
}

void RescueRobot::turn(int angle) {
  // Calculate delay-based turning (this is a simple approximation)
  int turnTime = abs(angle) * 10; // Adjust factor as needed for your robot
  if (angle > 0) {
    // Turn right
    setMotorSpeed(150, -150);
  } else {
    // Turn left
    setMotorSpeed(-150, 150);
  }
  delay(turnTime);
  setMotorSpeed(0, 0);
}

void RescueRobot::returnToStart() {
  // Example high-level routine to return to the starting zone
  Serial.println("Returning to start...");
  turn(180);       // Turn around
  moveForward(50); // Move forward a set distance (adjust as needed)
  // You can extend this with path planning using the map data
}

void RescueRobot::updateMap() {
  // Example: Print a message. You can extend this to update an array/grid.
  Serial.println("Mapping the field...");
}

void RescueRobot::displayMap() {
  // Output map data via Serial for debugging
  Serial.println("Displaying map data...");
}

void RescueRobot::lineFollow() {
  // Basic line following using the TCRT5000 sensor.
  int sensorValue = analogRead(LINE_SENSOR_PIN);
  const int threshold = 500; // Adjust based on calibration
  
  // If sensor reading is below threshold, assume the line is detected on one side
  if (sensorValue < threshold) {
    // Adjust motors to steer towards the line: slow left motor to turn right
    setMotorSpeed(120, 150);
  } else {
    // If sensor reading is higher, adjust to steer left
    setMotorSpeed(150, 120);
  }
  // Note: Call lineFollow() repeatedly in loop() for continuous tracking.
}

bool RescueRobot::detectBlock() {
  // Use the TCRT5000 sensor to detect if a block is in front
  int sensorValue = analogRead(LINE_SENSOR_PIN);
  const int threshold = 500; // Calibrate as needed
  return (sensorValue > threshold);
}

int RescueRobot::identifyBlock() {
  // Placeholder function to identify block type
  Serial.println("Identifying block...");
  // Return values could be:
  // 1 for red block, 2 for green block, 3 for brown block
  return 1; // Dummy value; update with real sensor logic if available.
}

void RescueRobot::pickBlock() {
  // Sequence to pick up a block using the 4DOF grappler
  Serial.println("Picking up block...");
  
  // Rotate grappler base to align with the block
  setGrapplerBase(90);
  delay(300);
  
  // Extend the arm toward the block
  setGrapplerArm(90);
  delay(300);
  
  // Lower the lift to bring the claw down
  setGrapplerLift(45);
  delay(300);
  
  // Close the claw to grab the block
  setGrapplerClaw(90); // Assume 90 degrees is closed
  delay(500);
}

void RescueRobot::releaseBlock() {
  // Sequence to release the block
  Serial.println("Releasing block...");
  
  // Open the claw to let go of the block
  setGrapplerClaw(0); // Assume 0 degrees is open
  delay(500);
  
  // Reset the lift, arm, and base to initial positions
  setGrapplerLift(90);
  delay(300);
  setGrapplerArm(0);
  delay(300);
  setGrapplerBase(0);
  delay(300);
}

void RescueRobot::setGrapplerBase(int angle) {
  grapplerBase.write(angle);
}

void RescueRobot::setGrapplerArm(int angle) {
  grapplerArm.write(angle);
}

void RescueRobot::setGrapplerLift(int angle) {
  grapplerLift.write(angle);
}

void RescueRobot::setGrapplerClaw(int angle) {
  graplerClaw.write(angle);
}

void RescueRobot::calibrateCompass() {
  // Placeholder for magnetometer calibration
  Serial.println("Calibrating compass...");
  // Add calibration routines for QMC5883L as needed.
}

int RescueRobot::getHeading() {
  // Read and process data from the magnetometer to compute heading
  int heading = 0;
  // Add real sensor reading and processing code here.
  return heading;
}

void RescueRobot::setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Set motor speeds using PWM. Adjust the logic based on your motor driver.
  analogWrite(LEFT_MOTOR_PIN, abs(leftSpeed));
  analogWrite(RIGHT_MOTOR_PIN, abs(rightSpeed));
  // For direction control, additional pins may be needed.
}
