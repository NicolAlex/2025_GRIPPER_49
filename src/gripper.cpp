#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

// Forward declarations so the functions are known before use
bool sendMessage(const char* msg);
bool readCommand(char* buffer, size_t length);

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode

bool Gripper::setupGripper() {
    uint8_t attachResult = gripperStepper.attach(STEP_PIN, DIR_PIN);
    if (attachResult == 0) {
        Serial.println("Failed to attach stepper motor.");
        return false;
    }
    gripperStepper.attachEnable(ENABLE_PIN, 100, LOW); // Attach enable pin (ENABLE_PIN) with 100ms enable delay; active LOW
    return true;
}

void Gripper::stepperMove() {
    static int speedMicroAdjusted = 0;
    static int stepsMicroAdjusted = 0;
    // Adjust speed based on microstepping mode
    switch (microSteppingMode) {
        case 2: // Half step
            speedMicroAdjusted = speed * 2;
            stepsMicroAdjusted = steps * 2;
            break;
        case 4: // Quarter step
            speedMicroAdjusted = speed * 4;
            stepsMicroAdjusted = steps * 4; 
            break;
        case 8: // Eighth step
            speedMicroAdjusted = speed * 8;
            stepsMicroAdjusted = steps * 8;
            break;
        case 16: // Sixteenth step
            speedMicroAdjusted = speed * 16;
            stepsMicroAdjusted = steps * 16;
            break;
        default:
            speedMicroAdjusted = speed;
            stepsMicroAdjusted = steps;
            break;
    }
    gripperStepper.setSpeed(speedMicroAdjusted); // Set speed in rpm*10
    gripperStepper.writeSteps(stepsMicroAdjusted); // Move specified steps
}

void Gripper::setOrigin() {
    disableStepper();  // Allow manual movement
    sendMessage("close gripper manually to set origin and press ENTER");
    static char commandBuffer[32];  // Buffer for serial input
    while (!readCommand(commandBuffer, sizeof(commandBuffer))) {
        // Wait for ENTER press
    }
    sendMessage("command received, setting origin...");
    enableStepper();
    gripperStepper.setZero(); // Set current position as origin
    sendMessage("origin set.");
}

void Gripper::setMicroStepping(int steppingMode) {
    switch (steppingMode) {
        case 2: // Half step
            digitalWrite(MS1_PIN, LOW);
            digitalWrite(MS2_PIN, HIGH);
            break;
        case 4: // Quarter step
            digitalWrite(MS1_PIN, HIGH);
            digitalWrite(MS2_PIN, LOW);
            break;
        case 8: // Eighth step
            digitalWrite(MS1_PIN, LOW);
            digitalWrite(MS2_PIN, LOW);
            break;
        case 16: // Sixteenth step
            digitalWrite(MS1_PIN, HIGH);
            digitalWrite(MS2_PIN, HIGH);
            break;
        default:
            //sendMessage("Invalid microstepping mode. Use 1, 2, 4, or 8.");
            return;
    }
    this->microSteppingMode = steppingMode;
    // sendMessage("Microstepping mode set.");
}

void Gripper::moveUntilClosed() {

}

void Gripper::enableStepper() {
    digitalWrite(ENABLE_PIN, LOW); // Assuming LOW enables the stepper
}

void Gripper::disableStepper() {
    digitalWrite(ENABLE_PIN, HIGH); // Assuming HIGH disables the stepper
}

bool sendMessage(const char* msg) {
    Serial.println(msg);
    return true;
}

bool readCommand(char* buffer, size_t length) {
    size_t bytesRead = Serial.readBytesUntil('\n', buffer, length - 1);
    buffer[bytesRead] = '\0';
    return bytesRead > 0;
}

int Gripper::getCommand() {
    static int commandReceived = 0;
    static char commandBuffer[32];
    if (readCommand(commandBuffer, sizeof(commandBuffer))) {
        // Process the command
    }
    if (strcmp(commandBuffer, "dosteps") == 0) {
        commandReceived = CMD_DO_STEPS;
    } else if (strcmp(commandBuffer, "rotate") == 0) {
        commandReceived = CMD_ROTATE;
    } else if (strcmp(commandBuffer, "setorigin") == 0) {
        commandReceived = CMD_SET_ORIGIN;
    } else if (strcmp(commandBuffer, "setmicrostepping") == 0) {
        commandReceived = CMD_SET_MICROSTEPPING;
    } else if (strcmp(commandBuffer, "moveuntilclosed") == 0) {
        commandReceived = CMD_MOVE_UNTIL_CLOSED;
    } else if (strcmp(commandBuffer, "enablestepper") == 0) {
        commandReceived = CMD_ENABLE_STEPPER;
    } else if (strcmp(commandBuffer, "disablestepper") == 0) {
        commandReceived = CMD_DISABLE_STEPPER;
    } else {
        sendMessage("Unknown command.");
        return -1;
    }
    return commandReceived;
}

void Gripper::testStepCmd() {
    static char commandBuffer[32];
    sendMessage("Enter speed (rpm*10): ");
    if (readCommand(commandBuffer, sizeof(commandBuffer))) {
        speed = atoi(commandBuffer);
        Serial.print(speed);
        sendMessage("Enter micro steps: ");
        if (readCommand(commandBuffer, sizeof(commandBuffer))) {
            microSteppingMode = atoi(commandBuffer);
            setMicroStepping(microSteppingMode);
            Serial.print(microSteppingMode);
        }
        sendMessage("Enter steps: ");
        if (readCommand(commandBuffer, sizeof(commandBuffer))) {
            steps = atoi(commandBuffer);
            Serial.print(steps);
        }
    }
}

void Gripper::testRotateCmd() {
    static char commandBuffer[32];
    sendMessage("Enter speed (rpm*10): ");
    if (readCommand(commandBuffer, sizeof(commandBuffer))) {
        speed = atoi(commandBuffer);
        Serial.print(speed);
        sendMessage("Enter micro steps: ");
        if (readCommand(commandBuffer, sizeof(commandBuffer))) {
            microSteppingMode = atoi(commandBuffer);
            setMicroStepping(microSteppingMode);
            Serial.print(microSteppingMode);
        }
        sendMessage("Enter number of rotations: ");
        if (readCommand(commandBuffer, sizeof(commandBuffer))) {
            static int rotations = atoi(commandBuffer);
            steps = rotations * 200; // Assuming 200 steps per revolution
            Serial.print(steps);
        }
    }
}
    