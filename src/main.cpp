#include <Arduino.h>
//#include <MobaTools.h>
#include "gripper.h"
#include <PS4Controller.h>

#ifdef GET_MAC_ADDRESS
#include <esp_bt_main.h>
#include <esp_bt_device.h>

void setup()
{
  Serial.begin(115200);
  PS4.begin();
  const uint8_t* address = esp_bt_dev_get_address();
  char str[100];
  sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x", address[0],address[1],address[2],address[3],address[4],address[5]);
  Serial.println(str);
}

void loop()
{

}
#endif


#ifdef SERVO_TEST_MODE

MoToServo testServo;

void setup() {
    Serial.begin(115200);
    pinMode(SERVO_PIN, OUTPUT);
    testServo.attach(SERVO_PIN);
    Serial.println("Servo Test Mode: Rotating servo between positions.");
}

void loop() {
    // Sweep servo from min to max position
    for (int pos = 0; pos <= 180; pos += 5) {
        testServo.write(pos);
        Serial.print("Servo position: ");
        Serial.println(pos);
        delay(500);
    }
    
    // Sweep back from max to min
    for (int pos = 180; pos >= 0; pos -= 5) {
        testServo.write(pos);
        Serial.print("Servo position: ");
        Serial.println(pos);
        delay(500);
    }
}

#endif

#ifdef OP_MODE
Gripper gripper;

constexpr int PS4_STATUS_SIZE = BUTTON_SHARE + 1;
int PS4_status[PS4_STATUS_SIZE] = {0};

void PS4_update() {
    if (PS4_status[CONNEXION_STATE] == 0) {
        for (int i = 0; i < PS4_STATUS_SIZE; i++)
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

    if (PS4_status[BUTTON_OPTIONS] == 0) { // button released by user
        PS4_status[BUTTON_OPTIONS] = PS4.Options();
    }
    if (PS4.Options() == 0) {
        PS4_status[BUTTON_OPTIONS] = 0; // button released
    }

    if (PS4_status[BUTTON_SHARE] == 0) { // button released by user
        PS4_status[BUTTON_SHARE] = PS4.Share();
    }
    if (PS4.Share() == 0) {
        PS4_status[BUTTON_SHARE] = 0; // button released
    }
}

void PS4_sendData() {

    static int lastUpdateTime = 0;
    if (millis() - lastUpdateTime > 10) {
        lastUpdateTime = millis();

        if (PS4_status[CONNEXION_STATE] == 0) {
            return;
        }

        static const int LED_IDLE[3] = {218, 86, 255};
        static const int LED_ARMED[3] = {0, 255, 100};
        static const int LED_UNIFORM_MOVE[3] = {255, 189, 0};
        static const int LED_GRIP[3] = {255, 0, 0};
        static const int LED_DEFAULT[3] = {255, 160, 160};

        if (gripper.getfsmState() == STATE_IDLE) {
            PS4.setLed(LED_IDLE[R], LED_IDLE[G], LED_IDLE[B]);
            PS4.setRumble(0, 0);
        } else if (gripper.getfsmState() == STATE_ARM) {
            PS4.setLed(LED_ARMED[R], LED_ARMED[G], LED_ARMED[B]);
            PS4.setRumble(0, 0);
        } else if (gripper.getfsmState() == STATE_UNIFORM_MOVE) {
            PS4.setLed(LED_UNIFORM_MOVE[R], LED_UNIFORM_MOVE[G], LED_UNIFORM_MOVE[B]);
            PS4.setRumble(100, 0);
        } else if (gripper.getfsmState() == STATE_GRIP) {
            PS4.setLed(LED_GRIP[R], LED_GRIP[G], LED_GRIP[B]);
            PS4.setRumble(0, 0);
        } else {
            PS4.setLed(LED_DEFAULT[R], LED_DEFAULT[G], LED_DEFAULT[B]);
            PS4.setRumble(0, 0);
        }

        PS4.sendToController();
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
    pinMode(SERVO_PIN, OUTPUT);

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
    
    PS4.begin("c8:c9:a3:c7:8d:7e");
    
    Serial.println("Ready.");
}



void loop() {

    serial_cmdHandler(&gripper, PS4_status);
    PS4_cmdHandler(&gripper, PS4_status);
    PS4_sendData();

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
#endif


