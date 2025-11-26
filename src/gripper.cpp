#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode


//================================================================================================================
// Constructor: Initialize gripper with default values
//================================================================================================================
Gripper::Gripper() {
    MS1state = LOW; // Default microstepping state for MS1
    MS2state = LOW; // Default microstepping state for MS2
    enabled = false;
    microSteppingMode = 2; // Default microstepping mode (2x)
    speed = 0;
    pos = 0;
    finalPos = 0;
    lastStep = 0;
    step = 0;
}


//================================================================================================================
// FSM loop: Handles state machine transitions and state-specific behaviors
//================================================================================================================
void Gripper::fsm_loop() {
    switch (fsmState) {
        case STATE_IDLE:
            finalPos = pos; // Hold position
            stepperDisable();
            break;

        case STATE_STEPPER_SPEED_TEST:
            // Non-blocking speed test: ramps speed up to MAX, holds 3s, then ramps down
            static int testLastTime = 0;
            static bool increasing = true;
            if (millis() - testLastTime >= 100) { // every 100 ms
                if (increasing) {
                    speed += (int)(MAX_STEPPER_SPEED / 100); // increase speed
                    if (speed >= MAX_STEPPER_SPEED) {
                        speed = MAX_STEPPER_SPEED;
                        increasing = false;
                        testLastTime = millis() + 3000; // hold for 3 seconds
                    }
                } else {
                    speed -= (int)(MAX_STEPPER_SPEED / 100); // decrease speed
                    if (speed <= 0) {
                        speed = 0;
                        increasing = true;
                        testLastTime = millis() + 3000; // hold for 3 seconds
                    }
                }
                stepperSetDir(1, speed, microSteppingMode);
                testLastTime = millis();
            }
            break;

        default:
            // Handle unknown state
            break;
    }
}

//================================================================================================================
// Updates stepper position and executes movement commands
// Called every loop iteration to sync logical position with hardware and drive motor
//================================================================================================================
void Gripper::stepperUpdate() {
    digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH); // Assuming active LOW
    if (!enabled) {
        return; // Stepper is disabled, do nothing
    }
    static int stepIncrement = 0; // persistent variable to hold required step delta
    setMicroSteps(); // apply current micro-stepping pin configuration
    step = gripperStepper.readSteps(); // read absolute step count from the stepper driver
    // update logical position (convert hardware steps to user steps by dividing out micro-stepping)
    pos += (int)((step - lastStep) / microSteppingMode);
    // align lastStep to the nearest micro-step boundary we've just consumed
    lastStep = step - ((step - lastStep) % microSteppingMode);
    stepIncrement = finalPos - pos; // compute how many user-steps remain to reach finalPos
    if (stepIncrement != 0) {
        // request the stepper to move the remaining steps at configured speed
        stepperMoveSteps(stepIncrement, speed);
    }
}

//================================================================================================================
// Trapezoidal motion profile computer: calculates target speed based on distance to goal
// Provides smooth acceleration, constant cruise speed, and deceleration ramps
//================================================================================================================
void Gripper::PPM_computer() {
    int32_t distanceToTarget = finalPos - pos;
    int32_t absDistance = abs(distanceToTarget);
    
    if (absDistance == 0) {
        speed = 0;
        return;
    }
    
    // Motion profile parameters
    const int MIN_SPEED = 20;          // Minimum speed to avoid motor stalling
    const int CRUISE_SPEED = MAX_STEPPER_SPEED;
    const int ACCEL_INCREMENT = 5;     // Speed increment per update for smooth acceleration
    
    int targetSpeed = 0;
    
    if (absDistance > 2 * ACCEL_STEPS) {
        // Acceleration phase: gradually increase speed
        targetSpeed = min(CRUISE_SPEED, speed + ACCEL_INCREMENT);
    } else {
        // Transition or deceleration zone
        int halfDistance = absDistance / 2;
        if (absDistance > halfDistance) {
            targetSpeed = map(halfDistance, 0, ACCEL_STEPS, MIN_SPEED, CRUISE_SPEED);
        } else {
            targetSpeed = map(absDistance, 0, halfDistance, MIN_SPEED, CRUISE_SPEED);
        }
    }
    
    if (absDistance <= ACCEL_STEPS) {
        // Deceleration phase: slow down as approaching target
        int decelerationSpeed = map(absDistance, 0, ACCEL_STEPS, MIN_SPEED, CRUISE_SPEED);
        targetSpeed = min(targetSpeed, decelerationSpeed);
    }
    
    targetSpeed = constrain(targetSpeed, MIN_SPEED, CRUISE_SPEED);
    speed = targetSpeed;
}

//================================================================================================================
// Commands the stepper motor to move a specific number of steps at given speed
// Converts logical steps to hardware steps based on microstepping mode
//================================================================================================================
void Gripper::stepperMoveSteps(int steps, int runSpeed) {
    gripperStepper.setSpeed(runSpeed * microSteppingMode);
    gripperStepper.doSteps(steps * microSteppingMode);
}

//================================================================================================================
// Sets continuous rotation direction and speed for the stepper motor
// Direction: 0=stop, >0=forward, <0=backward. Updates finalPos for continuous motion.
//================================================================================================================
void Gripper::stepperSetDir(int direction, int runSpeed, int microSteps) {
    speed = runSpeed;
    microSteppingMode = microSteps;
    if (direction == 0) {
        finalPos = 0;
    } else if (direction > 0) {
        finalPos = pos + MAX_STEP_DIR;
    } else if (direction < 0) {
        finalPos = pos - MAX_STEP_DIR;
    }
}

//================================================================================================================
// Enables the stepper motor driver (pulls ENABLE pin LOW for A4988/DRV8825)
//================================================================================================================
void Gripper::stepperEnable() {
    enabled = true;
}

//================================================================================================================
// Disables the stepper motor driver to save power and prevent holding torque
//================================================================================================================
void Gripper::stepperDisable() {
    enabled = false;
}

//================================================================================================================
// Initializes the gripper hardware: attaches stepper motor pins and reads initial position
// Returns true if setup successful, false otherwise
//================================================================================================================
bool Gripper::setupGripper() {
    uint8_t attachResult = gripperStepper.attach(STEP_PIN, DIR_PIN);
    delay(10);
    lastStep = gripperStepper.readSteps();
    if (attachResult == 0) {
        sendMessage("Failed to attach stepper motor.");
        return false;
    }
    return true;
}

//================================================================================================================
// Sets the current FSM state for the gripper
//================================================================================================================
void Gripper::setState(int newState) {
    fsmState = newState;
}

//================================================================================================================
// Interactive calibration: waits for user to manually close gripper, then sets position to zero
// Blocks until user confirms via Serial input
//================================================================================================================
void Gripper::stepperSetOrigin() {
    stepperDisable();
    sendMessage("Close gripper to set origin and enter \"ok\".");
    while(1) {
        while(!Serial.available()) {
            // Wait for user input
            delay(10);
        }
        char* msg = readMessage();
        if (msg != nullptr && strcmp(msg, "ok") == 0) {
            pos = 0;
            finalPos = 0;
            sendMessage("Origin set.");
            stepperEnable();
            return;
        }
    }
}

//================================================================================================================
// Configures MS1 and MS2 pins to set microstepping resolution (2, 4, 8, or 16)
// Inline for performance in stepperUpdate loop
//================================================================================================================
inline void Gripper::setMicroSteps() {
    static bool MS1state = LOW;
    static bool MS2state = LOW;
    switch (microSteppingMode) {
        case 4:
            MS1state = LOW;
            MS2state = HIGH;
            break;
        case 2:
            MS1state = HIGH;
            MS2state = LOW;
            break;
        case 8:
            MS1state = LOW;
            MS2state = LOW;
            break;
        case 16:
            MS1state = HIGH;
            MS2state = HIGH;
            break;
        default:
            MS1state = LOW;
            MS2state = LOW;
            break;
    }
    digitalWrite(MS1_PIN, MS1state);
    digitalWrite(MS2_PIN, MS2state);
}

//================================================================================================================
// Returns current logical position in user-defined steps
//================================================================================================================
int32_t Gripper::getPosition() {
    return pos;
}

//================================================================================================================
// Returns target position the gripper is moving towards
//================================================================================================================
int32_t Gripper::getFinalPosition() {
    return finalPos;
}

//================================================================================================================
// Returns raw hardware step count from the stepper driver
//================================================================================================================
int32_t Gripper::getStepCount() {
    return step;
}

//================================================================================================================
// Returns current microstepping mode (2, 4, 8, or 16)
//================================================================================================================
int32_t Gripper::getMicroSteppingMode() {
    return microSteppingMode;
}

//================================================================================================================
// Sets new target position for the gripper to move to
//================================================================================================================
void Gripper::setPosition(int32_t newPos) {
    finalPos = newPos;
}

//================================================================================================================
// Manually sets the motor speed (useful for testing or override)
//================================================================================================================
void Gripper::setSpeed(int newSpeed) {
    speed = newSpeed;
}

//================================================================================================================
// Updates microstepping resolution (2, 4, 8, or 16 microsteps per full step)
//================================================================================================================
void Gripper::setMicroSteppingMode(int newMicroSteps) {
    microSteppingMode = newMicroSteps;
}

//================================================================================================================
// Converts gripper finger length to motor rotation angle via linear interpolation
// Uses lookup table interpolSample for non-linear kinematics
//================================================================================================================
float Gripper::interpolToAngle(float length) {
    static float ratio = 0.0;
    if (length <= interpolSample[0][0]) {
        return interpolSample[0][1];
    }
    if (length >= interpolSample[INTERPOL_SAMPLES - 1][0]) {
        return interpolSample[INTERPOL_SAMPLES - 1][1];
    }
    for (int i = 0; i < INTERPOL_SAMPLES - 1; i++) {
        if (length >= interpolSample[i][0] && length <= interpolSample[i + 1][0]) {
            ratio = (length - interpolSample[i][0]) / (interpolSample[i + 1][0] - interpolSample[i][0]);
            return interpolSample[i][1] + ratio * (interpolSample[i + 1][1] - interpolSample[i][1]);
        }
    }
    return interpolSample[INTERPOL_SAMPLES - 1][1];
}

//================================================================================================================
// Converts motor rotation angle to gripper finger length via linear interpolation
// Inverse of interpolToAngle for kinematic feedback
//================================================================================================================
float Gripper::interpolToLength(float angle) {
    static float ratio = 0.0;
    if (angle <= interpolSample[0][1]) {
        return interpolSample[0][0];
    }
    if (angle >= interpolSample[INTERPOL_SAMPLES - 1][1]) {
        return interpolSample[INTERPOL_SAMPLES - 1][0];
    }
    for (int i = 0; i < INTERPOL_SAMPLES - 1; i++) {
        if (angle >= interpolSample[i][1] && angle <= interpolSample[i + 1][1]) {
            ratio = (angle - interpolSample[i][1]) / (interpolSample[i + 1][1] - interpolSample[i][1]);
            return interpolSample[i][0] + ratio * (interpolSample[i + 1][0] - interpolSample[i][0]);
        }
    }
    return interpolSample[INTERPOL_SAMPLES - 1][0];
}

//================================================================================================================
// Debug command dispatcher: processes serial commands to control gripper during testing
// Handles setup, calibration, movement, and configuration commands
//================================================================================================================
void debugCommandHandler(int cmd, Gripper* gripper) {
    if (cmd != -1 && gripper != nullptr) {
        switch (cmd) {
            case CMD_SETUP_STEPPER:
                sendMessage("Setting up stepper...");
                if (gripper->setupGripper()) {
                    sendMessage("Stepper setup complete.");
                } else {
                    sendMessage("Stepper setup failed.");
                }
                break;
            case CMD_SET_ORIGIN:
                sendMessage("Setting origin...");
                gripper->stepperSetOrigin();
                sendMessage("Origin set.");
                break;
            case CMD_SET_MICROSTEPPING:
            {
                sendMessage("Enter new microstepping mode (2, 4, 8, 16):");
                while(!Serial.available()) {
                    delay(10);
                }
                char* msg = readMessage();
                if (msg != nullptr) {
                    int newMicroSteps = atoi(msg);
                    if (newMicroSteps == 2 || newMicroSteps == 4 || newMicroSteps == 8 || newMicroSteps == 16) {
                        gripper->setMicroSteppingMode(newMicroSteps);
                        sendMessage("Microstepping set to ");
                        Serial.println(gripper->getMicroSteppingMode());
                    } else {
                        sendMessage("Invalid microstepping mode.");
                    }
                }
                break;
            }
            case CMD_MOVE_UNTIL_CLOSED:
                sendMessage("Moving until closed...");
                // Implement move until closed logic here
                sendMessage("Movement complete.");
                break;

            case CMD_ENABLE_STEPPER:
                sendMessage("Enabling stepper...");
                gripper->stepperEnable();
                sendMessage("Stepper enabled.");
                break;

            case CMD_DISABLE_STEPPER:
                sendMessage("Disabling stepper...");
                gripper->stepperDisable();
                sendMessage("Stepper disabled.");
                break;

            case CMD_DO_STEPS:
                sendMessage("Enter position to move to:");
                while(!Serial.available()) {
                    delay(10);
                }
                {
                    char* msg = readMessage();
                    if (msg != nullptr) {
                        int newPos = atoi(msg);
                        gripper->setPosition(newPos);
                        sendMessage("Moving to position ");
                        Serial.println(gripper->getPosition());
                    }
                }
                break;

            case CMD_ROTATE:
                sendMessage("Rotating...");
                // Implement rotation logic here
                sendMessage("Rotation complete.");
                break;

            default:
                sendMessage("Unknown command.");
                break;
        }
    }
}

//================================================================================================================
// Sends a message string over Serial for debugging/user feedback
//================================================================================================================
void sendMessage(const char* msg) {
    Serial.println(msg);
}

//================================================================================================================
// Reads a line of text from Serial input, handling CR/LF properly
// Returns nullptr if no data available, empty string for ENTER key, or message text
//================================================================================================================
char* readMessage() {
    static char buffer[100];
    if (!Serial.available()) {
        return nullptr;
    }

    // If the next byte(s) are only CR/LF (ENTER), consume them and return an empty string
    int peeked = Serial.peek();
    if (peeked == '\n' || peeked == '\r') {
        while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
            Serial.read();
        }
        buffer[0] = '\0';
        return buffer;
    }

    // Read a line up to '\n'
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    if (len > 0 && buffer[len - 1] == '\r') {
        len--;
    }
    buffer[len] = '\0';
    return buffer;
}

//================================================================================================================
// Parses Serial input string and returns corresponding command code
// Returns -1 if no command or unknown command
//================================================================================================================
int getCommand() {
    char* msg = readMessage();
    if (msg == nullptr) {
        return -1; // No command available
    }
    if (strcmp(msg, "set_origin") == 0) {
        return CMD_SET_ORIGIN;
    } else if (strcmp(msg, "set_microstepping") == 0) {
        return CMD_SET_MICROSTEPPING;
    } else if (strcmp(msg, "move_until_closed") == 0) {
        return CMD_MOVE_UNTIL_CLOSED;
    } else if (strcmp(msg, "enable_stepper") == 0) {
        return CMD_ENABLE_STEPPER;
    } else if (strcmp(msg, "disable_stepper") == 0) {
        return CMD_DISABLE_STEPPER;
    } else if (strcmp(msg, "do_steps") == 0) {
        return CMD_DO_STEPS;
    } else if (strcmp(msg, "rotate") == 0) {
        return CMD_ROTATE;
    } else if (strcmp(msg, "setup_stepper") == 0) {
        return CMD_SETUP_STEPPER;
    }
    return -1; // Unknown command
}

//================================================================================================================
// Animates status LEDs in a rotating pattern when enabled
// Shows first LED solid when disabled (idle state indicator)
//================================================================================================================
void statusLedBlinking(bool enabled) {
    static unsigned long lastTime = 0;
    static int ledIndex = 0;
    if (enabled) {
        if (millis() - lastTime >= 50) { // Blink every 50 ms
            // Turn off all LEDs
            for (int i = 0; i < 4; i++) {
                digitalWrite(ledArray[i], LOW);
            }
            // Turn on the current LED
            digitalWrite(ledArray[ledIndex], HIGH);
            // Move to the next LED
            ledIndex = (ledIndex + 1) % 4;
            lastTime = millis();
        }
    }
    else {
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED3_PIN, LOW);
        digitalWrite(LED4_PIN, LOW);
    }
}

//================================================================================================================
// Controls buzzer with timed beep: setup=true starts beep, false checks/stops after duration
// Non-blocking implementation for integration in main loop
//================================================================================================================
void buzzerBeep(int duration, bool setup) {
    static unsigned long beepStartTime = 0;
    static bool isBeeping = false;

    if (setup) {
        digitalWrite(BUZZER_PIN, HIGH);
        beepStartTime = millis();
        isBeeping = true;
        return;
    }

    if (!isBeeping && setup == true) {
        // Start the beep
        digitalWrite(BUZZER_PIN, HIGH);
        beepStartTime = millis();
        isBeeping = true;
    } else {
        // Check if the duration has elapsed
        if (millis() - beepStartTime >= duration) {
            digitalWrite(BUZZER_PIN, LOW);
            isBeeping = false;
        }
    }
}