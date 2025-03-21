# RescueRobotLib Documentation

Welcome to the RescueRobotLib documentation! This guide will explain how to use the library that controls our rescue robot. The library is designed to make it easier for you to command the robot without worrying about the low-level details. In simple terms, you can tell the robot what to do (like move forward, turn, or pick up a block) and the library will handle the motor and sensor details.

## Overview

**RescueRobotLib** is a collection of functions written in Arduino C/C++ that control your robot’s movements, sensors, and the grappling mechanism (grappler). This library lets you:
- Drive the robot forward, backward, or turn.
- Follow a line on the game board to help the robot find blocks.
- Update and display a simple map of the game board.
- Use a 4DOF (four degrees of freedom) grappler to pick up and drop blocks.
- Use a magnetometer (compass) to help with navigation.

## Hardware Components

The library is built for a robot that uses the following hardware:
- **4-Wheel Chassis:** The base that moves the robot.
- **Arduino Mega:** The microcontroller that runs the code.
- **4DOF Grappler:** 
  - **Base Servo:** Rotates the grappler.
  - **Arm Servo:** Moves the arm that reaches out.
  - **Lift Servo:** Raises or lowers the grappler.
  - **Claw Servo:** Opens and closes to grab a block.
- **SG90 Servo Motors:** Used in the grappler.
- **TCRT5000 Reflective Optocoupler:** A sensor that can detect lines on the board and help target blocks.
- **QMC5883L Magnetometer:** Acts like a compass to help the robot know which way it is facing.
- **Line on the Board:** A special line that the robot can follow, which helps it navigate and target blocks.

## Getting Started

### Installation

1. **Include the Library in Your Project:**  
   Add the header file to your Arduino sketch:
   ```cpp
   #include "RescueRobotLib.h"
   ```

2. **Initialization:**  
   Create a robot object and initialize it in the `setup()` function:
   ```cpp
   #include "RescueRobotLib.h"
   
   RescueRobot robot;
   
   void setup() {
     Serial.begin(9600);  // For debugging messages
     robot.begin();       // Set up motors, sensors, and servos
   }
   ```

3. **Running Commands:**  
   In the `loop()` function, call the library functions to perform actions. For example, the robot can follow a line, pick up a block, and return to the start point.

## Library Functions

Below is an explanation of each function provided by the library and what it does. The language is kept simple so you can understand without needing technical training.

### 1. Basic Movement

- **`moveForward(int distance)`**  
  **What it does:** Moves the robot forward by a set distance (measured in centimeters).  
  **How to use it:**  
  ```cpp
  robot.moveForward(50);  // Moves forward 50 cm
  ```

- **`turn(int angle)`**  
  **What it does:** Rotates the robot. A positive angle turns the robot to the right, and a negative angle turns it to the left.  
  **How to use it:**  
  ```cpp
  robot.turn(90);   // Turns right 90 degrees
  robot.turn(-90);  // Turns left 90 degrees
  ```

- **`returnToStart()`**  
  **What it does:** This high-level function tells the robot to navigate back to its starting position.  
  **How to use it:**  
  ```cpp
  robot.returnToStart();
  ```

### 2. Mapping and Line Following

- **`updateMap()`**  
  **What it does:** Uses the line sensor and other inputs to update a simple map of the competition field. Think of this as the robot “drawing” the area it’s moving through.  
  **How to use it:**  
  ```cpp
  robot.updateMap();
  ```

- **`displayMap()`**  
  **What it does:** Prints the map data to the Serial Monitor for debugging.  
  **How to use it:**  
  ```cpp
  robot.displayMap();
  ```

- **Line Following Note:**  
  The robot uses the TCRT5000 sensor to detect the line on the board. This helps in guiding the robot to find and target the blocks. The function `detectBlock()` uses this sensor to decide if a block is in front of the robot.

### 3. Block Handling Functions

- **`detectBlock()`**  
  **What it does:** Checks if a block is present in front of the robot by reading the TCRT5000 sensor.  
  **How to use it:**  
  ```cpp
  if (robot.detectBlock()) {
    // A block is detected
  }
  ```

- **`identifyBlock()`**  
  **What it does:** Figures out what type of block the robot is seeing (for example, red, green, or brown). This can be based on markings or colors.  
  **How to use it:**  
  ```cpp
  int blockType = robot.identifyBlock();
  ```

- **`pickBlock()`**  
  **What it does:** Activates the grappler to grab a block. It uses four servos for different movements:
  - **Base Servo:** Rotates the grappler.
  - **Arm Servo:** Extends the arm.
  - **Lift Servo:** Raises or lowers the arm.
  - **Claw Servo:** Opens/closes to grab the block.
  
  **How to use it:**  
  ```cpp
  robot.pickBlock();
  ```

- **`releaseBlock()`**  
  **What it does:** Releases the block by opening the claw servo.  
  **How to use it:**  
  ```cpp
  robot.releaseBlock();
  ```

### 4. Grappler Detailed Control (Advanced)

If you want more control over the grappler, you can later add functions to adjust each of the four servos individually. For example, you could add:

- **`setGrapplerBase(int angle)`**  
  Moves the base servo to a specific angle.

- **`setGrapplerArm(int angle)`**  
  Adjusts the arm servo to extend or retract.

- **`setGrapplerLift(int angle)`**  
  Raises or lowers the grappler.

- **`setGrapplerClaw(int angle)`**  
  Opens or closes the claw.

For now, the `pickBlock()` and `releaseBlock()` functions wrap these movements into one simple command.

### 5. Compass and Navigation

- **`calibrateCompass()`**  
  **What it does:** Prepares the magnetometer (compass) so that the robot knows its orientation (which way it is facing).  
  **How to use it:**  
  This is usually called during initialization in `begin()`.  
  ```cpp
  robot.calibrateCompass();
  ```

- **`getHeading()`**  
  **What it does:** Returns the current direction (in degrees) that the robot is facing.  
  **How to use it:**  
  ```cpp
  int heading = robot.getHeading();
  Serial.println(heading);
  ```

### 6. Internal Helper Function

- **`setMotorSpeed(int leftSpeed, int rightSpeed)`**  
  **What it does:** Directly controls the left and right motors. It is used by other functions to move the robot.  
  **Note:** This function is meant for internal use and usually doesn’t need to be called directly in your main program.

## Example Usage

Below is an example sketch that shows how to combine these functions to make the robot perform a simple task:

```cpp
#include "RescueRobotLib.h"

RescueRobot robot;

void setup() {
  Serial.begin(9600);
  robot.begin();  // Initialize motors, sensors, and servos
}

void loop() {
  // Update the map and check for a block
  robot.updateMap();
  
  if (robot.detectBlock()) {
    // Identify the block type (red, green, or brown)
    int blockType = robot.identifyBlock();
    Serial.print("Block type: ");
    Serial.println(blockType);
    
    // Pick up the block using the grappler
    robot.pickBlock();
    
    // Navigate to a drop-off zone based on the block type
    if (blockType == 1) {
      Serial.println("Moving to Hospital Zone");
    } else if (blockType == 2) {
      Serial.println("Moving to Refuge Zone");
    } else {
      Serial.println("Moving to Default Zone");
    }
    
    // Move to the zone (example movements)
    robot.moveForward(30);
    robot.turn(90);
    robot.moveForward(20);
    
    // Release the block
    robot.releaseBlock();
    
    // Return to the start for the next task
    robot.returnToStart();
  }
  
  // Wait a moment before trying again
  delay(2000);
}
```

## Final Notes

- **Modularity:**  
  The library is built so that all complex hardware control is hidden behind simple function calls. This means you only need to think about high-level tasks (like “go pick up a block”) without getting into technical details.

- **Customization:**  
  If you need more control (for example, controlling each servo on the grappler individually), you can extend the library with additional functions. The current functions are a good starting point that meet the competition’s rules.

- **Safety and Compliance:**  
  The functions ensure that the robot follows the rules of the challenge by using fixed patterns for movement and block handling. This helps reduce mistakes during the competition.

We hope this documentation helps you get started with your robot programming. Enjoy building and modifying your robot to perform rescue missions in the challenge!

---

This guide is written to be understandable by everyone, even if you’re not an expert in robotics. If you have any questions, feel free to ask your mentor or refer to additional Arduino tutorials for more details on basic programming and electronics.
