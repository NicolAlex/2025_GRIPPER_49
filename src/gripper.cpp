#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

void sendMessage(const char* msg);
char* readMessage();

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode


//================================================================================================================
void Gripper::stepperUpdate() {
    if (!enabled) {
        stepperMoveSteps(0, 0);
        return;
    }
    stepperMoveSteps(finalPos - pos, speed);
    step = gripperStepper.readSteps();
    pos += (step - lastStep)/microSteppingMode;
    lastStep = step;
    setMicroSteps(); // Ensure microstepping mode is maintained
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