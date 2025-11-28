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

    commandHandler(&gripper);
    gripper.fsm_loop();
    gripper.stepperUpdate();

    gripController.readPressure();
    static float pressure = 0.0f;
    pressure = gripController.getPressure();

    // compute loop frequency
    static unsigned long lastLoopTime = 0;
    static float loopFrequency = 0.0f;
    static int loopCount = 0;
    loopCount++;
    if (millis() - lastLoopTime >= 1000) { // every second
        loopFrequency = loopCount / ((millis() - lastLoopTime) / 1000.0f);
        loopCount = 0;
        lastLoopTime = millis();
    }
    
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime >= 300 && gripper.verboseEnabled) { // every 0.1 second
        lastStatusTime = millis();
        gripper.verbose();
        Serial.print("Loop Frequency: ");
        Serial.println(loopFrequency);
        Serial.print("Pressure: ");
        Serial.println(pressure);
    }

    delay(1);
}


