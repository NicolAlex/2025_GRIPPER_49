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
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    // testStepper.attach(STEP_PIN, DIR_PIN);
    // delay(100);
    // int initialSteps = testStepper.readSteps();
    // Serial.println("Initial Steps: ");
    // Serial.println(initialSteps);
    gripper.setupGripper();
    gripper.stepperEnable();
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinMode(LED4_PIN, OUTPUT);
    pinMode(LED5_PIN, OUTPUT);
    pinMode(LED6_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    digitalWrite(LED4_PIN, LOW);
    digitalWrite(LED5_PIN, LOW);
    digitalWrite(LED6_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    gripper.setSpeed(0);
}



void loop() {
    /*
    static int cmd = -1;
    cmd = getCommand();
    debugCommandHandler(cmd, &gripper);

    //gripper.fsm_loop();

    gripper.PPM_computer();
    gripper.stepperUpdate();

    static uint lastTime = millis();

    if (millis() - lastTime >= 100) {
        lastTime = millis();
        Serial.print("Position: ");
        Serial.println(gripper.getPosition());
        Serial.print("Step Count: ");
        Serial.println(gripper.getStepCount());
        Serial.print("current Speed: ");
        Serial.println(gripper.getSpeed());
        Serial.println("----");
    }

    static int diff = 0;
    static int lastDiff = 0;
    if (diff != 0) {
        statusLedBlinking(true);
    }
    else {
        statusLedBlinking(false);
    }
    diff = gripper.getFinalPosition() - gripper.getPosition();
    if (diff == 0 && lastDiff != 0) {
        buzzerBeep(50, true); // Start beep
    }
    buzzerBeep(50, false); // Check and stop beep if duration elapsed
    lastDiff = diff;

    delay(1);
    */


    // Test code for readMessage function
    static char cmd[32], arg1[32], arg2[32];
    
    if (readMessage(cmd, arg1, arg2)) {
        Serial.println("=== Command Received ===");
        Serial.print("Command: ");
        Serial.println(cmd);
        Serial.print("Argument 1: ");
        Serial.println(strlen(arg1) > 0 ? arg1 : "(empty)");
        Serial.print("Argument 2: ");
        Serial.println(strlen(arg2) > 0 ? arg2 : "(empty)");
        Serial.println("========================");
    }
    
    delay(10);
}


