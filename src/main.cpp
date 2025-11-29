#include <Arduino.h>
//#include <MobaTools.h>
#include "gripper.h"
#include <PS4Controller.h>

Gripper gripper;

// MoToStepper testStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode

int PS4_status[13] = {0};

void PS4_update()
{
    if (PS4_status[CONNEXION_STATE] == 0) {
        for (int i = 0; i < 13; i++)
            PS4_status[i] = 0;
        return;
    }

    // Logic to avoid multiple detections of button presses
    if (PS4_status[BUTTON_CROSS] == 0) { // button released by user
        PS4_status[BUTTON_CROSS] = PS4.Cross();
    }
    if (PS4.Cross() == 0) {
        PS4_status[BUTTON_CROSS] = 0; // button released
    }

    if (PS4_status[BUTTON_CIRCLE] == 0) { // button released by user
        PS4_status[BUTTON_CIRCLE] = PS4.Circle();
    }
    if (PS4.Circle() == 0) {
        PS4_status[BUTTON_CIRCLE] = 0; // button released
    }

    if (PS4_status[BUTTON_TRIANGLE] == 0) { // button released by user
        PS4_status[BUTTON_TRIANGLE] = PS4.Triangle();
    }
    if (PS4.Triangle() == 0) {
        PS4_status[BUTTON_TRIANGLE] = 0; // button released
    }

    if (PS4_status[BUTTON_SQUARE] == 0) { // button released by user
        PS4_status[BUTTON_SQUARE] = PS4.Square();
    }
    if (PS4.Square() == 0) {
        PS4_status[BUTTON_SQUARE] = 0; // button released
    }

    if (PS4_status[BUTTON_UP] == 0) { // button released by user
        PS4_status[BUTTON_UP] = PS4.Up();
    }
    if (PS4.Up() == 0) {
        PS4_status[BUTTON_UP] = 0; // button released
    }

    if (PS4_status[BUTTON_DOWN] == 0) { // button released by user
        PS4_status[BUTTON_DOWN] = PS4.Down();
    }
    if (PS4.Down() == 0) {
        PS4_status[BUTTON_DOWN] = 0; // button released
    }

    if (PS4_status[BUTTON_LEFT] == 0) { // button released by user
        PS4_status[BUTTON_LEFT] = PS4.Left();
    }
    if (PS4.Left() == 0) {
        PS4_status[BUTTON_LEFT] = 0; // button released
    }

    if (PS4_status[BUTTON_RIGHT] == 0) { // button released by user
        PS4_status[BUTTON_RIGHT] = PS4.Right();
    }
    if (PS4.Right() == 0) {
        PS4_status[BUTTON_RIGHT] = 0; // button released
    }

    if (PS4_status[BUTTON_L1] == 0) { // button released by user
        PS4_status[BUTTON_L1] = PS4.L1();
    }
    if (PS4.L1() == 0) {
        PS4_status[BUTTON_L1] = 0; // button released
    }

    if (PS4_status[BUTTON_R1] == 0) { // button released by user
        PS4_status[BUTTON_R1] = PS4.R1();
    }
    if (PS4.R1() == 0) {
        PS4_status[BUTTON_R1] = 0; // button released
    }

    if (PS4_status[BUTTON_L2] == 0) { // button released by user
        PS4_status[BUTTON_L2] = PS4.L2();
    }
    if (PS4.L2() == 0) {
        PS4_status[BUTTON_L2] = 0; // button released
    }

    if (PS4_status[BUTTON_R2] == 0) { // button released by user
        PS4_status[BUTTON_R2] = PS4.R2();
    }
    if (PS4.R2() == 0) {
        PS4_status[BUTTON_R2] = 0; // button released
    }
}

void onConnect()
{
  Serial.println("Connected!.");
  PS4_status[CONNEXION_STATE] = 1;
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
  PS4_status[CONNEXION_STATE] = 0;
}

void setup() {
    Serial.begin(115200);

    // Initialize GPIO pins
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);

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

    // Initialize gripper
    gripper.setupGripper();
    gripper.stepperEnable();

    // initialize Bluetooth
    delay(1000); // Wait for a second to ensure Serial is ready
    Serial.println("serial test");
    Serial.flush(); // Ensure message is sent
    
    PS4.attach(PS4_update);
    Serial.println("Attach done");
    
    PS4.attachOnConnect(onConnect);
    Serial.println("OnConnect attached");
    
    PS4.attachOnDisconnect(onDisConnect);
    Serial.println("OnDisconnect attached");
    
    Serial.println("Starting PS4 controller...");
    Serial.flush();
    
    PS4.begin("20:43:a8:e5:6b:8e");
    
    Serial.println("Ready.");
}



void loop() {

    commandHandler(&gripper);
    PS4_cmdHandler(PS4_status, &gripper);

    gripper.fsm_loop(PS4_status);
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
        gripper.verbose(PS4_status);
        Serial.print("Loop Frequency: ");
        Serial.println(loopFrequency);
        Serial.print("Pressure: ");
        Serial.println(pressure);
    }

    delay(1);
}


