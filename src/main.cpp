#include <Arduino.h>
//#include <MobaTools.h>
#include "gripper.h"

// #define DIR_PIN 13
// #define STEP_PIN 12
// #define ENABLE_PIN 14
// #define MS1_PIN 27
// #define MS2_PIN 26

Gripper gripper;

// MoToStepper testStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode

void setup() {
    Serial.begin(115200);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    // testStepper.attach(STEP_PIN, DIR_PIN);
    // delay(100);
    // int initialSteps = testStepper.readSteps();
    // Serial.println("Initial Steps: ");
    // Serial.println(initialSteps);
    gripper.setupGripper();
    gripper.stepperEnable();
}

void loop() {
    static int cmd = -1;
    cmd = getCommand();
    if (cmd != -1) {
        switch (cmd) {
            case CMD_SETUP_STEPPER:
                sendMessage("Setting up stepper...");
                if (gripper.setupGripper()) {
                    sendMessage("Stepper setup complete.");
                } else {
                    sendMessage("Stepper setup failed.");
                }
                break;
            case CMD_SET_ORIGIN:
                sendMessage("Setting origin...");
                gripper.stepperSetOrigin();
                sendMessage("Origin set.");
                break;
            case CMD_SET_MICROSTEPPING:
            {
                sendMessage("Enter new microstepping mode (2, 4, 8, 16):");
                while(!Serial.available()) {
                    // Wait for user input
                    delay(10);
                }
                char* msg = readMessage();
                if (msg != nullptr) {
                    int newMicroSteps = atoi(msg);
                    if (newMicroSteps == 2 || newMicroSteps == 4 || newMicroSteps == 8 || newMicroSteps == 16) {
                        gripper.setMicroSteppingMode(newMicroSteps);
                        sendMessage("Microstepping set to ");
                        Serial.println(gripper.getMicroSteppingMode());
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
                gripper.stepperEnable();
                sendMessage("Stepper enabled.");
                break;

            case CMD_DISABLE_STEPPER:
                sendMessage("Disabling stepper...");
                gripper.stepperDisable();
                sendMessage("Stepper disabled.");
                break;

            case CMD_DO_STEPS:
                sendMessage("Enter position to move to:");
                while(!Serial.available()) {
                    // Wait for user input
                    delay(10);
                }
                {
                    char* msg = readMessage();
                    if (msg != nullptr) {
                        int newPos = atoi(msg);
                        gripper.setPosition(newPos);
                        sendMessage("Moving to position ");
                        Serial.println(gripper.getPosition());
                    }
                }
                sendMessage("Enter rotation speed:");
                while(!Serial.available()) {
                    // Wait for user input
                    delay(10);
                }
                {
                    char* msg = readMessage();
                    if (msg != nullptr) {
                        int newSpeed = atoi(msg);
                        gripper.setSpeed(newSpeed);
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

        gripper.stepperUpdate();
        Serial.println(gripper.getPosition());
        Serial.println(gripper.getStepCount());
        Serial.println("----");

    delay(100);
}