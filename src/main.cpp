#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.cpp"

Gripper gripper;

void setup() {
    Serial.begin(115200);
    // Your setup code here
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
}

void loop() {
    // Your main code here
    while(!Serial);
    gripper.setupGripper();
    static int cmd = -1;
    cmd = gripper.getCommand();
    if (cmd == CMD_DO_STEPS) {
        gripper.testStepCmd();
        gripper.stepperMove();
    } else if (cmd == CMD_ROTATE) {
        gripper.testRotateCmd();
        gripper.stepperMove();
    } else {
        sendMessage("No valid command received.");
    }
}