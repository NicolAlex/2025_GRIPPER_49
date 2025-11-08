#include <Arduino.h>
#include <MobaTools.h>

#include "gripper.h"


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
    
}