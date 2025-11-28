#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode

// convertion functions def
static inline float rpmToStepsPerSec(int rpm);
static inline float rpmToStepsPerSec(int rpm);

static inline float rpmToStepsPerSec(int rpm);
static inline int stepsPerSecToRpm(float stepsPerSec);


//================================================================================================================
// Constructor: Initialize gripper with default values
//================================================================================================================
Gripper::Gripper() {
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
// FSM loop: Handles state machine transitions and state-specific behaviors
//================================================================================================================
void Gripper::fsm_loop() {

    return; // Temporarily disabled
}

//================================================================================================================
// Updates stepper position and executes movement commands
// Called every loop iteration to sync logical position with hardware and drive motor
//================================================================================================================
void Gripper::stepperUpdate() {
    digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH); // Assuming active LOW
    if (!enabled) {
        return; // Stepper is disabled, do nothing
    }
    static int stepIncrement = 0; // persistent variable to hold required step delta
    setMicroSteps(); // apply current micro-stepping pin configuration
    step = gripperStepper.readSteps(); // read absolute step count from the stepper driver
    // update logical position (convert hardware steps to user steps by dividing out micro-stepping)
    pos += (int)((step - lastStep) / microSteppingMode);
    // align lastStep to the nearest micro-step boundary we've just consumed
    lastStep = step - ((step - lastStep) % microSteppingMode);
    stepIncrement = finalPos - pos; // compute how many user-steps remain to reach finalPos
    if (stepIncrement != 0) {
        // request the stepper to move the remaining steps at configured speed
        stepperMoveSteps(stepIncrement, speed);
    }
}

//================================================================================================================
// Trapezoidal motion profile computer: calculates target speed based on distance to goal
// Provides smooth acceleration, constant cruise speed, and deceleration ramps
//================================================================================================================
void Gripper::PPM_computer() {
    // Early exit if on target
    if (finalPos == pos) {
        speed = 0;
        return;
    }

    // Constants in steps/sec
    const float vmax_stepsSec = rpmToStepsPerSec(MAX_STEPPER_SPEED);
    const float vmin_stepsSec = rpmToStepsPerSec(MIN_RUN_SPEED);
    const float a = static_cast<float>(ACCELERATION); // steps/sec^2 (ensure ACCELERATION matches this)

    // Persistent profile state
    static int32_t lastTarget = 0;
    static float v_start = 0.0f;
    static float v_peak = 0.0f;
    static uint32_t tAccel_ms = 0;
    static uint32_t tCruise_ms = 0;
    static uint32_t tDecel_ms = 0;
    static uint32_t profileStart_ms = 0;

    // Time tracking
    uint32_t now_ms = millis();
    uint32_t tSinceStart_ms = now_ms - profileStart_ms;

    // Start new profile if target changed
    if (finalPos != lastTarget) {
        int32_t dist_steps = abs(finalPos - pos); // full steps
        v_start = rpmToStepsPerSec(speed);        // current speed in steps/sec

        // Distance needed for full accel + decel (from vStart to vmax then to 0)
        float dNeeded = ((vmax_stepsSec * vmax_stepsSec - v_start * v_start) / (2.0f * a)) +
                        ((vmax_stepsSec * vmax_stepsSec) / (2.0f * a));

        if (dist_steps < dNeeded) {
            // Triangular profile: solve peak speed
            // dist = (Vp^2 - vStart^2)/(2a) + (Vp^2)/(2a) => Vp^2 = a*dist + 0.5*vStart^2
            v_peak = sqrtf(a * dist_steps + 0.5f * v_start * v_start);
            if (v_peak < v_start) {
                // Only decelerate if already above achievable peak
                v_peak = v_start;
                tAccel_ms = 0;
                tDecel_ms = (uint32_t)(v_peak / a * 1000.0f);
                tCruise_ms = 0;
            } else {
                tAccel_ms = (uint32_t)(((v_peak - v_start) / a) * 1000.0f);
                tDecel_ms = (uint32_t)((v_peak / a) * 1000.0f);
                tCruise_ms = 0;
            }
        } else {
            // Trapezoidal profile
            v_peak = vmax_stepsSec;
            tAccel_ms = (uint32_t)(((v_peak - v_start) / a) * 1000.0f);
            tDecel_ms = (uint32_t)((v_peak / a) * 1000.0f);

            // Distance used in accel + decel
            float dAccel = (v_peak * v_peak - v_start * v_start) / (2.0f * a);
            float dDecel = (v_peak * v_peak) / (2.0f * a);
            float dCruise = dist_steps - dAccel - dDecel;
            if (dCruise < 0) dCruise = 0;
            tCruise_ms = (uint32_t)((dCruise / v_peak) * 1000.0f);
        }

        profileStart_ms = now_ms;
        tSinceStart_ms = 0;
        lastTarget = finalPos;
    }

    // Compute instantaneous speed (steps/sec)
    float vCurrent;
    uint32_t tTotal_ms = tAccel_ms + tCruise_ms + tDecel_ms;

    if (tSinceStart_ms >= tTotal_ms) {
        vCurrent = 0.0f;
    } else if (tSinceStart_ms >= (tAccel_ms + tCruise_ms)) {
        // Decel phase
        uint32_t tIntoDecel = tSinceStart_ms - (tAccel_ms + tCruise_ms);
        vCurrent = v_peak - a * (tIntoDecel / 1000.0f);
        if (vCurrent < 0) vCurrent = 0;
    } else if (tSinceStart_ms >= tAccel_ms) {
        // Cruise
        vCurrent = v_peak;
    } else {
        // Accel
        vCurrent = v_start + a * (tSinceStart_ms / 1000.0f);
        if (vCurrent > v_peak) vCurrent = v_peak;
    }

    // Enforce min speed (except when near stop)
    if (vCurrent > 0 && vCurrent < vmin_stepsSec) {
        vCurrent = vmin_stepsSec;
    }

    // Update rpm-facing member
    speed = stepsPerSecToRpm(vCurrent);
}


//================================================================================================================
// Commands the stepper motor to move a specific number of steps at given speed
// Converts logical steps to hardware steps based on microstepping mode
//================================================================================================================
void Gripper::stepperMoveSteps(int steps, int runSpeed) {
    gripperStepper.setSpeed(runSpeed * microSteppingMode * 10);
    gripperStepper.doSteps(steps * microSteppingMode);
}

//================================================================================================================
// Sets continuous rotation direction and speed for the stepper motor
// Direction: 0=stop, >0=forward, <0=backward. Updates finalPos for continuous motion.
//================================================================================================================
void Gripper::stepperSetDir(int direction, int runSpeed, int microSteps) {
    speed = runSpeed;
    microSteppingMode = microSteps;
    if (direction == 0) {
        finalPos = 0;
    } else if (direction > 0) {
        finalPos = pos + MAX_STEP_DIR;
    } else if (direction < 0) {
        finalPos = pos - MAX_STEP_DIR;
    }
}

//================================================================================================================
// Enables the stepper motor driver
//================================================================================================================
void Gripper::stepperEnable() {
    enabled = true;
}

//================================================================================================================
// Disables the stepper motor driver to save power and prevent holding torque
//================================================================================================================
void Gripper::stepperDisable() {
    enabled = false;
}

//================================================================================================================
// Initializes the gripper hardware: attaches stepper motor pins and reads initial position
// Returns true if setup successful, false otherwise
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

//================================================================================================================
// Sets the current FSM state for the gripper
//================================================================================================================
void Gripper::setState(int newState) {
    fsmState = newState;
}

//================================================================================================================
// Interactive calibration: waits for user to manually close gripper, then sets position to zero
// Blocks until user confirms via Serial input
//================================================================================================================
void Gripper::stepperSetOrigin() {
    return;; // Temporarily disabled
}

//================================================================================================================
// Measures fruit size based on current gripper position
//================================================================================================================
void Gripper::getFruitSize() {
    static float openingLength = 0.0f;
    openingLength = InterpolToLength(static_cast<float>(pos));
    if (openingLength < FRUIT_SIZE_THRESHOLD) {
        fruitSize = FRUIT_SMALL; // Small fruit
    } else {
        fruitSize = FRUIT_LARGE; // Large fruit
    }
}

//================================================================================================================
// Configures MS1 and MS2 pins to set microstepping resolution (2, 4, 8, or 16)
// Inline for performance in stepperUpdate loop
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
// Returns current logical position in user-defined steps
//================================================================================================================
int32_t Gripper::getPosition() {
    return pos;
}

//================================================================================================================
// Returns target position the gripper is moving towards
//================================================================================================================
int32_t Gripper::getFinalPosition() {
    return finalPos;
}

//================================================================================================================
// Returns raw hardware step count from the stepper driver
//================================================================================================================
int32_t Gripper::getStepCount() {
    return step;
}

//================================================================================================================
// Returns current microstepping mode (2, 4, 8, or 16)
//================================================================================================================
int32_t Gripper::getMicroSteppingMode() {
    return microSteppingMode;
}

//================================================================================================================
// Returns current motor speed in steps per minute
//================================================================================================================
int Gripper::getSpeed() {
    return speed;
}

//================================================================================================================
// Sets new target position for the gripper to move to
//================================================================================================================
void Gripper::setPosition(int32_t newPos) {
    finalPos = newPos;
}

//================================================================================================================
// Manually sets the motor speed (useful for testing or override)
//================================================================================================================
void Gripper::setSpeed(int newSpeed) {
    speed = newSpeed;
}

//================================================================================================================
// Updates microstepping resolution (2, 4, 8, or 16 microsteps per full step)
//================================================================================================================
void Gripper::setMicroSteppingMode(int newMicroSteps) {
    microSteppingMode = newMicroSteps;
}

//================================================================================================================
// Prepares servo motor for operation (attach to pin, set initial position)
//================================================================================================================
void Gripper::servoSetup() {
    // Placeholder for servo setup code
}


//================================================================================================================
bool Gripper::servoRotate() {
    return true;
}

//================================================================================================================
// Converts gripper finger length to motor rotation angle via linear interpolation
// Uses lookup table interpolSample for non-linear kinematics
//================================================================================================================
float InterpolToAngle(float length) {
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
    return interpolSample[INTERPOL_SAMPLES - 1][1];
}

//================================================================================================================
// Converts motor rotation angle to gripper finger length via linear interpolation
// Inverse of interpolToAngle for kinematic feedback
//================================================================================================================
float InterpolToLength(float angle) {
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
    return interpolSample[INTERPOL_SAMPLES - 1][0];
}

//================================================================================================================
// Debug command dispatcher: processes serial commands to control gripper during testing
// Handles setup, calibration, movement, and configuration commands
//================================================================================================================
void debugCommandHandler(int cmd, Gripper* gripper) {
    if (cmd != -1 && gripper != nullptr) {
        switch (cmd) {
            case CMD_SETUP_STEPPER:
                sendMessage("Setting up stepper...");
                if (gripper->setupGripper()) {
                    sendMessage("Stepper setup complete.");
                } else {
                    sendMessage("Stepper setup failed.");
                }
                break;
            case CMD_SET_ORIGIN:
                sendMessage("Setting origin...");
                gripper->stepperSetOrigin();
                sendMessage("Origin set.");
                break;
            case CMD_SET_MICROSTEPPING:
            {
                sendMessage("Enter new microstepping mode (2, 4, 8, 16):");
                while(!Serial.available()) {
                    delay(10);
                }
                char* msg = readMessage();
                if (msg != nullptr) {
                    int newMicroSteps = atoi(msg);
                    if (newMicroSteps == 2 || newMicroSteps == 4 || newMicroSteps == 8 || newMicroSteps == 16) {
                        gripper->setMicroSteppingMode(newMicroSteps);
                        sendMessage("Microstepping set to ");
                        Serial.println(gripper->getMicroSteppingMode());
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
                gripper->stepperEnable();
                sendMessage("Stepper enabled.");
                break;

            case CMD_DISABLE_STEPPER:
                sendMessage("Disabling stepper...");
                gripper->stepperDisable();
                sendMessage("Stepper disabled.");
                break;

            case CMD_DO_STEPS:
                sendMessage("Enter position to move to:");
                while(!Serial.available()) {
                    delay(10);
                }
                {
                    char* msg = readMessage();
                    if (msg != nullptr) {
                        int newPos = atoi(msg);
                        gripper->setPosition(newPos);
                        sendMessage("Moving to position ");
                        Serial.println(gripper->getPosition());
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
// Sends a message string over Serial for debugging/user feedback
//================================================================================================================
void sendMessage(const char* msg) {
    Serial.println(msg);
}

//================================================================================================================
// Reads a line of text from Serial input, handling CR/LF properly
// Parses command format: "<command> <arg1> <arg2>" or "<command> <arg1>" or "<command>"
// Returns true if a complete command was parsed, false if still reading or no data
// Non-blocking: accumulates chars until newline, then parses into outCmd/outArg1/outArg2
//================================================================================================================
bool readMessage(char* outCmd, char* outArg1, char* outArg2) {
    static char buffer[128];
    static int bufferIndex = 0;
    
    // Clear output buffers
    outCmd[0] = '\0';
    outArg1[0] = '\0';
    outArg2[0] = '\0';
    
    // Read available characters
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Handle newline (command complete)
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                buffer[bufferIndex] = '\0'; // Null-terminate
                
                // Parse the buffer into command and arguments
                char* token = strtok(buffer, " ");
                if (token != nullptr) {
                    strncpy(outCmd, token, 31);
                    outCmd[31] = '\0';
                    
                    token = strtok(nullptr, " ");
                    if (token != nullptr) {
                        strncpy(outArg1, token, 31);
                        outArg1[31] = '\0';
                        
                        token = strtok(nullptr, " ");
                        if (token != nullptr) {
                            strncpy(outArg2, token, 31);
                            outArg2[31] = '\0';
                        }
                    }
                }
                
                bufferIndex = 0; // Reset buffer
                return true; // Command ready
            }
            // Ignore empty lines (just CR/LF)
            continue;
        }
        
        // Accumulate characters
        if (bufferIndex < sizeof(buffer) - 1) {
            buffer[bufferIndex++] = c;
        } else {
            // Buffer overflow - reset
            bufferIndex = 0;
            sendMessage("Error: Command too long");
        }
    }
    
    return false; // No complete command yet
}

//================================================================================================================
// Parses Serial input string and returns corresponding command code
// Returns -1 if no command or unknown command
//================================================================================================================
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

//================================================================================================================
// Animates status LEDs in a rotating pattern when enabled
// Shows first LED solid when disabled (idle state indicator)
//================================================================================================================
void statusLedBlinking(bool enabled) {
    static unsigned long lastTime = 0;
    static int ledIndex = 0;
    if (enabled) {
        if (millis() - lastTime >= 50) { // Blink every 50 ms
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

//================================================================================================================
// Controls buzzer with timed beep: setup=true starts beep, false checks/stops after duration
// Non-blocking implementation for integration in main loop
//================================================================================================================
void buzzerBeep(int duration, bool setup) {
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

// Helper conversions (keep near top of file or before class methods)
static inline float rpmToStepsPerSec(int rpm) {
    return (rpm * 200.0f) / 60.0f; // full steps/sec (microstepping excluded intentionally)
}
static inline int stepsPerSecToRpm(float sps) {
    return (int) ((sps * 60.0f) / 200.0f);
}