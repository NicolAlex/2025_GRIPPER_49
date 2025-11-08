#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode


//================================================================================================================
void Gripper::stepperUpdate() {
    static int stepIncrement = 0;                     // persistent variable to hold required step delta
    setMicroSteps();                                  // apply current micro-stepping pin configuration
    step = gripperStepper.readSteps();                // read absolute step count from the stepper driver
    // update logical position (convert hardware steps to user steps by dividing out micro-stepping)
    pos += (int)((step - lastStep) / microSteppingMode);
    // align lastStep to the nearest micro-step boundary we've just consumed
    lastStep = step - ((step - lastStep) % microSteppingMode);
    stepIncrement = finalPos - pos;                   // compute how many user-steps remain to reach finalPos
    if (stepIncrement != 0) {
        // request the stepper to move the remaining steps at configured speed
        stepperMoveSteps(stepIncrement, speed);
    }
}

//================================================================================================================
void Gripper::stepperMoveSteps(int steps, int runSpeed) {
    gripperStepper.setSpeed(runSpeed * microSteppingMode);
    gripperStepper.doSteps(steps * microSteppingMode);
}

//================================================================================================================
void Gripper::stepperSetDir(int direction, int runSpeed, int microSteps) {
    speed = runSpeed;
    microSteppingMode = microSteps;
    if (direction == 0) {
        finalPos = 0;
    } else if (direction > 0) {
        finalPos = MAX_STEP_DIR;
    } else if (direction < 0) {
        finalPos = -MAX_STEP_DIR;
    }
}

//================================================================================================================
void Gripper::stepperEnable() {
    digitalWrite(ENABLE_PIN, LOW); // Assuming active LOW
    enabled = true;
}

//================================================================================================================
void Gripper::stepperDisable() {
    digitalWrite(ENABLE_PIN, HIGH); // Assuming active LOW
    enabled = false;
}

//================================================================================================================
bool Gripper::setupGripper() {
    uint8_t attachResult = gripperStepper.attach(STEP_PIN, DIR_PIN);
    delay(10);
    lastStep = gripperStepper.readSteps();
    if (attachResult == 0) {
        sendMessage("Failed to attach stepper motor.");
        return false;
    }
    gripperStepper.attachEnable(ENABLE_PIN, 100, LOW); // Attach enable pin (ENABLE_PIN) with 100ms enable delay; active LOW
    return true;
}

void Gripper::stepperSetOrigin() {
    stepperDisable();
    sendMessage("Close gripper to set origin and press ENTER.");
    while (true) {
        char* msg = readMessage();
        if (msg != nullptr && strcmp(msg, "\n") == 0) {
            break;
        }
    }
    pos = 0;
    finalPos = 0;
    sendMessage("Origin set.");
    stepperEnable();
}

//================================================================================================================
void Gripper::setMicroSteps() {
    static bool MS1state = LOW;
    static bool MS2state = LOW;
    switch (microSteppingMode) {
        case 2:
            MS1state = LOW;
            MS2state = HIGH;
            break;
        case 4:
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
int32_t Gripper::getPosition() {
    return pos;
}

//================================================================================================================
int32_t Gripper::getStepCount() {
    return step;
}

//================================================================================================================
int32_t Gripper::getMicroSteppingMode() {
    return microSteppingMode;
}

//================================================================================================================
void Gripper::setPosition(int32_t newPos) {
    finalPos = newPos;
}

//================================================================================================================
void Gripper::setSpeed(int newSpeed) {
    speed = newSpeed;
}

//================================================================================================================
void Gripper::setMicroSteppingMode(int newMicroSteps) {
    microSteppingMode = newMicroSteps;
}

















void sendMessage(const char* msg) {
    Serial.println(msg);
}

char* readMessage() {
    static char buffer[100];
    if (Serial.available()) {
        size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0'; // Null-terminate the string
        return buffer;
    }
    return nullptr;
}

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