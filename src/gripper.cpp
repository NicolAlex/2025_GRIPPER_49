#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode


Gripper::Gripper() {
    // Constructor
    MS1state = LOW; // Default microstepping state for MS1
    MS2state = LOW; // Default microstepping state for MS2
    enabled = false;
    microSteppingMode = 2; // Default microstepping mode (2x)
    speed = 0;
    pos = 0;
    finalPos = 0;
    lastStep = 0;
    step = 0;
}

//================================================================================================================
void Gripper::stepperUpdate() {
    // enabling/disabling
    digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH); // Assuming active LOW
    static int stepIncrement = 0;                     // persistent variable to hold required step delta
    setMicroSteps();                                  // apply current micro-stepping pin configuration
    step = gripperStepper.readSteps();                // read absolute step count from the stepper driver
    // update logical position (convert hardware steps to user steps by dividing out micro-stepping)
    pos += (int)((step - lastStep) / microSteppingMode);
    // align lastStep to the nearest micro-step boundary we've just consumed
    lastStep = step - ((step - lastStep) % microSteppingMode);
    stepIncrement = finalPos - pos;                   // compute how many user-steps remain to reach finalPos
    if (stepIncrement != 0) {
        // request the stepper to move the remaining steps at configured speed
        stepperMoveSteps(stepIncrement, speed);
    }
}


void Gripper::comamandHandler(int cmd) {
    if (cmd != -1) {
        switch (cmd) {
            case CMD_SETUP_STEPPER:
                sendMessage("Setting up stepper...");
                if (setupGripper()) {
                    sendMessage("Stepper setup complete.");
                } else {
                    sendMessage("Stepper setup failed.");
                }
                break;
            case CMD_SET_ORIGIN:
                sendMessage("Setting origin...");
                stepperSetOrigin();
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
                        setMicroSteppingMode(newMicroSteps);
                        sendMessage("Microstepping set to ");
                        Serial.println(getMicroSteppingMode());
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
                stepperEnable();
                sendMessage("Stepper enabled.");
                break;

            case CMD_DISABLE_STEPPER:
                sendMessage("Disabling stepper...");
                stepperDisable();
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
                        setPosition(newPos);
                        sendMessage("Moving to position ");
                        Serial.println(getPosition());
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
                        setSpeed(newSpeed);
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
    //digitalWrite(ENABLE_PIN, LOW); // Assuming active LOW
    enabled = true;
}

//================================================================================================================
void Gripper::stepperDisable() {
    //digitalWrite(ENABLE_PIN, HIGH); // Assuming active LOW
    enabled = false;
}

//================================================================================================================
bool Gripper::setupGripper() {
    uint8_t attachResult = gripperStepper.attach(STEP_PIN, DIR_PIN);
    delay(10);
    lastStep = gripperStepper.readSteps();
    if (attachResult == 0) {
        sendMessage("Failed to attach stepper motor.");
        return false;
    }
    return true;
}

void Gripper::stepperSetOrigin() {
    stepperDisable();
    sendMessage("Close gripper to set origin and enter \"ok\".");
    while(1) {
        while(!Serial.available()) {
            // Wait for user input
            delay(10);
        }
        char* msg = readMessage();
        // Process both nullptr and empty string as valid ENTER key press
        if (msg != nullptr && strcmp(msg, "ok") == 0) {
            pos = 0;
            finalPos = 0;
            sendMessage("Origin set.");
            stepperEnable();
            return;
        }
    }
}

//================================================================================================================
inline void Gripper::setMicroSteps() {
    static bool MS1state = LOW;
    static bool MS2state = LOW;
    switch (microSteppingMode) {
        case 4:
            MS1state = LOW;
            MS2state = HIGH;
            break;
        case 2:
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
int32_t Gripper::getFinalPosition() {
    return finalPos;
}

//================================================================================================================
int32_t Gripper::getStepCount() {
    return step;
}

//================================================================================================================
int32_t Gripper::getMicroSteppingMode() {
    return microSteppingMode;
}

//================================================================================================================
void Gripper::setPosition(int32_t newPos) {
    finalPos = newPos;
}

//================================================================================================================
void Gripper::setSpeed(int newSpeed) {
    speed = newSpeed;
}

//================================================================================================================
void Gripper::setMicroSteppingMode(int newMicroSteps) {
    microSteppingMode = newMicroSteps;
}

//================================================================================================================
float Gripper::interpolToAngle(float length) {
    static float ratio = 0.0;
    if (length <= interpolSample[0][0]) {
        return interpolSample[0][1];
    }
    if (length >= interpolSample[INTERPOL_SAMPLES - 1][0]) {
        return interpolSample[INTERPOL_SAMPLES - 1][1];
    }
    for (int i = 0; i < INTERPOL_SAMPLES - 1; i++) {
        if (length >= interpolSample[i][0] && length <= interpolSample[i + 1][0]) {
            ratio = (length - interpolSample[i][0]) / (interpolSample[i + 1][0] - interpolSample[i][0]);
            return interpolSample[i][1] + ratio * (interpolSample[i + 1][1] - interpolSample[i][1]);
        }
    }
    // Fallback: return the last sample's angle to ensure a value is always returned
    return interpolSample[INTERPOL_SAMPLES - 1][1];
}

//================================================================================================================
float Gripper::interpolToLength(float angle) {
    static float ratio = 0.0;
    if (angle <= interpolSample[0][1]) {
        return interpolSample[0][0];
    }
    if (angle >= interpolSample[INTERPOL_SAMPLES - 1][1]) {
        return interpolSample[INTERPOL_SAMPLES - 1][0];
    }
    for (int i = 0; i < INTERPOL_SAMPLES - 1; i++) {
        if (angle >= interpolSample[i][1] && angle <= interpolSample[i + 1][1]) {
            ratio = (angle - interpolSample[i][1]) / (interpolSample[i + 1][1] - interpolSample[i][1]);
            return interpolSample[i][0] + ratio * (interpolSample[i + 1][0] - interpolSample[i][0]);
        }
    }
    // Fallback: return the last sample's length to ensure a value is always returned
    return interpolSample[INTERPOL_SAMPLES - 1][0];
}


















void sendMessage(const char* msg) {
    Serial.println(msg);
}

char* readMessage() {
    static char buffer[100];
    if (!Serial.available()) {
        return nullptr;
    }

    // If the next byte(s) are only CR/LF (ENTER), consume them and return an empty string
    int peeked = Serial.peek();
    if (peeked == '\n' || peeked == '\r') {
        // consume all leading CR/LF characters
        while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
            Serial.read();
        }
        buffer[0] = '\0';
        return buffer;
    }

    // Read a line up to '\n'
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    // strip trailing CR if present (handles CR+LF)
    if (len > 0 && buffer[len - 1] == '\r') {
        len--;
    }
    buffer[len] = '\0';
    return buffer;
}

int getCommand() {
    char* msg = readMessage();
    if (msg == nullptr) {
        return -1; // No command available
    }
    if (strcmp(msg, "set_origin") == 0) {
        return CMD_SET_ORIGIN;
    } else if (strcmp(msg, "set_microstepping") == 0) {
        return CMD_SET_MICROSTEPPING;
    } else if (strcmp(msg, "move_until_closed") == 0) {
        return CMD_MOVE_UNTIL_CLOSED;
    } else if (strcmp(msg, "enable_stepper") == 0) {
        return CMD_ENABLE_STEPPER;
    } else if (strcmp(msg, "disable_stepper") == 0) {
        return CMD_DISABLE_STEPPER;
    } else if (strcmp(msg, "do_steps") == 0) {
        return CMD_DO_STEPS;
    } else if (strcmp(msg, "rotate") == 0) {
        return CMD_ROTATE;
    } else if (strcmp(msg, "setup_stepper") == 0) {
        return CMD_SETUP_STEPPER;
    }
    return -1; // Unknown command
}






void statusLedBlinking(bool enabled) {
    static unsigned long lastTime = 0;
    static int ledIndex = 0;
    if (enabled) {
        if (millis() - lastTime >= 50) { // Blink every 25 ms
            // Turn off all LEDs
            for (int i = 0; i < 4; i++) {
                digitalWrite(ledArray[i], LOW);
            }
            // Turn on the current LED
            digitalWrite(ledArray[ledIndex], HIGH);
            // Move to the next LED
            ledIndex = (ledIndex + 1) % 4;
            lastTime = millis();
        }
    }

    else {
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED3_PIN, LOW);
        digitalWrite(LED4_PIN, LOW);
    }
}

void buzzerBeep(int duration, bool setup) { // if setup is true, start the beep, if false, stop after duration
    static unsigned long beepStartTime = 0;
    static bool isBeeping = false;

    if (setup) {
        digitalWrite(BUZZER_PIN, HIGH);
        beepStartTime = millis();
        isBeeping = true;
        return;
    }

    if (!isBeeping && setup == true) {
        // Start the beep
        digitalWrite(BUZZER_PIN, HIGH);
        beepStartTime = millis();
        isBeeping = true;
    } else {
        // Check if the duration has elapsed
        if (millis() - beepStartTime >= duration) {
            digitalWrite(BUZZER_PIN, LOW);
            isBeeping = false;
        }
    }
}