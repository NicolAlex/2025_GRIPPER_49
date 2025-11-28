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

    while(!Serial);
}



void loop() {
    gripper.fsm_loop();
    gripper.stepperUpdate();
    delay(10);
}


