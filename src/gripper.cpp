#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode

controller gripController;

// convertion functions def
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
    fsmState = STATE_INIT;
    maxOpSpeed = MAX_STEPPER_SPEED;
    minOpSpeed = 50;
    opAccel = 1000.0f; // steps/sec^2
    fruitSize = UNDEFINED_SIZE;
    verboseEnabled = false;

}


//================================================================================================================
// FSM loop: Handles state machine transitions and state-specific behaviors
//================================================================================================================
void Gripper::fsm_loop(int *PS4_status) {
    switch(fsmState) {
        case STATE_INIT:
            setupGripper();
            stepperDisable(); // ensure stepper is disabled at init
            fsmState = STATE_GOOFY;
            Serial.println("WARNING : Entering GOOFY state. Please calibrate the gripper.");
            break;
        case STATE_GOOFY:
            stepperDisable();
            break;
        case STATE_CALIBRATE_SERIAL:
            Serial.println("Starting calibration...");
            stepperSetOrigin_fromSerial();
            fsmState = STATE_ARM;
            break;
        case STATE_CALIBRATE_PS4:
            Serial.println("Starting calibration...");
            stepperSetOrigin_fromPS4(PS4_status);
            fsmState = STATE_ARM;
            break;
        case STATE_IDLE:
            stepperDisable();
            break;
        case STATE_UNIFORM_MOVE:
            // Uniform move state actions
            stepperEnable();
            if (PS4_status[BUTTON_LEFT] == 1 || PS4_status[BUTTON_LEFT] == -1) {
                stepperSetDir(1,100, microSteppingMode);
            } else if (PS4_status[BUTTON_RIGHT] == 1 || PS4_status[BUTTON_RIGHT] == -1) {
                stepperSetDir(-1,100, microSteppingMode);
            }
            else {
                stepperSetDir(0,0, microSteppingMode); // Stop gripper
                fsmState = STATE_ARM;
                finalPos = pos; // hold position
            }
            break;
        case STATE_ARM:
            stepperEnable();
            PPM_computer();
            break;
        case STATE_GRIP:
            // Grip state actions
            static int gripSpeed = 0;
            gripController.readPressure();
            gripSpeed = gripController.getOutputSpeed();
            if (gripController.checkSteadyState()) {
                fsmState = STATE_SORT;
                break;
            }
            microSteppingMode = 16; // best precision for gripping
            gripController.PID_computer();
            stepperSetDir( (gripSpeed >=0) ? 1 : -1, abs(gripSpeed), microSteppingMode);
            break;
        case STATE_SORT:
            // Sort state actions
            break;
        case STATE_MOVE_BOX:
            // Move box state actions
            break;
        case STATE_RELEASE:
            // Release state actions
            break;
        case STATE_DEBUG_STEPPER:
            verbose(PS4_status);
            break;
        case STATE_DEBUG_TRACK_PRESS:
            // Debug track pressure state actions
            break;
        case STATE_DEBUG_SERVO:
            // Debug servo state actions
            break;
        case STATE_DEBUG_PRINT_PRESS:
            // Debug print pressure state actions
            break;
        default:
            // Default case
            break;
    }

    return; // Temporarily disabled
}

//================================================================================================================
// Updates stepper position and executes movement commands
// Called every loop iteration to sync logical position with hardware and drive motor
//================================================================================================================
void Gripper::stepperUpdate() {
    digitalWrite(ENABLE_PIN, enabled ? LOW : HIGH); // Assuming active LOW
    if (!enabled) {
        finalPos = pos; // Stepper is disabled, hold
    }
    static float stepIncrement = 0.0f; // changed to float for fractional steps
    // Protect gripper from exceding mechanical limits
    if (finalPos > MAX_POSITION) {
        finalPos = MAX_POSITION;
    } else if (finalPos < MIN_POSITION) {
        finalPos = MIN_POSITION;
    }
    setMicroSteps(); // apply current micro-stepping pin configuration
    step = gripperStepper.readSteps(); // read absolute step count from the stepper driver
    
    // Calculate fractional position change based on microsteps
    float stepDelta = (float)(step - lastStep) / (float)microSteppingMode;
    pos += stepDelta; // pos is now float, accumulates fractional steps
    
    // Update lastStep to current hardware position
    lastStep = step;
    
    stepIncrement = finalPos - pos; // compute how many user-steps remain to reach finalPos
    if (abs(stepIncrement) > 0.001f) { // use small threshold instead of exact zero for floats
        // request the stepper to move the remaining steps at configured speed
        stepperMoveSteps((int)round(stepIncrement), speed);
    }
}

//================================================================================================================
// Trapezoidal motion profile computer: calculates target speed based on distance to goal
// Provides smooth acceleration, constant cruise speed, and deceleration ramps
//================================================================================================================
void Gripper::PPM_computer() {
    // Early exit if on target (use float comparison with tolerance)
    if (fabsf(finalPos - pos) < 0.001f) {
        speed = 0;
        return;
    }

    // steps/sec
    static float vmax_stepsSec = rpmToStepsPerSec(maxOpSpeed);
    static float vmin_stepsSec = rpmToStepsPerSec(minOpSpeed);
    static float a = opAccel; // steps/sec^2 (ensure ACCELERATION matches this)

    // Persistent profile state
    static float lastTarget = 0;
    static float v_start = 0.0f;
    static float v_peak = 0.0f;
    static uint32_t tAccel_ms = 0;
    static uint32_t tCruise_ms = 0;
    static uint32_t tDecel_ms = 0;
    static uint32_t profileStart_ms = 0;

    // Time tracking
    uint32_t now_ms = millis();
    uint32_t tSinceStart_ms = now_ms - profileStart_ms;

    // Start new profile if target changed (use float comparison)
    if (fabsf(finalPos - lastTarget) > 0.001f) {

        // update profile parameters
        vmax_stepsSec = rpmToStepsPerSec(maxOpSpeed);
        vmin_stepsSec = rpmToStepsPerSec(minOpSpeed);
        a = opAccel; // steps/sec^2

        float dist_steps = fabsf(finalPos - pos); // full steps (now supports fractional)
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
        finalPos = MAX_POSITION;
    } else if (direction < 0) {
        finalPos = MIN_POSITION;
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
        Serial.print("Failed to attach stepper motor.");
        return false;
    }
    Serial.println("Gripper initialized!");
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
void Gripper::stepperSetOrigin_fromSerial() {
    Serial.println("manually close the gripper and press enter when done");
    digitalWrite(ENABLE_PIN, HIGH); // enforce disable to allow manual movement
    enabled = false;
    while (Serial.available() == 0) {
        // wait for user input
        delay(100);
    }
    pos = 0;
    finalPos = 0;
    Serial.println("Calibration complete. current position set to zero.");
    digitalWrite(ENABLE_PIN, LOW); // re-enable stepper
    enabled = true;
    fsmState = STATE_ARM;
    Serial.println("Gripper armed !");
}

void Gripper::stepperSetOrigin_fromPS4(int *PS4_status) {
    Serial.println("manually close the gripper and press O when done");
    digitalWrite(ENABLE_PIN, HIGH); // enforce disable to allow manual movement
    enabled = false;
    while (*(PS4_status + BUTTON_CIRCLE) != 1) {
        // wait for user input
        delay(10);
    }
    *(PS4_status + BUTTON_CIRCLE) = -1; // reset button state
    pos = 0;
    finalPos = 0;
    Serial.println("Calibration complete. current position set to zero.");
    limitedBeep(100,4);
    digitalWrite(ENABLE_PIN, LOW); // re-enable stepper
    enabled = true;
    fsmState = STATE_ARM;
    Serial.println("Gripper armed !");
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

// ================================================================================================================
int Gripper::getfsmState() {
    return fsmState;
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
    if (newMicroSteps != 2 && newMicroSteps != 4 && newMicroSteps != 8 && newMicroSteps != 16) {
        Serial.println("Invalid microstepping mode. Must be 2, 4, 8, or 16.");
        microSteppingMode = 2;
        return;
    }
    microSteppingMode = newMicroSteps;
}

//================================================================================================================
// Sets maximum operational speed for motion profiling
//================================================================================================================
void Gripper::setMaxOpSpeed(int maxSpeed) {
    if (maxSpeed < 0) maxSpeed = 0;
    else if (maxSpeed > MAX_STEPPER_SPEED) maxSpeed = MAX_STEPPER_SPEED;
    else maxOpSpeed = maxSpeed;
}

//================================================================================================================
// Sets minimum operational speed for motion profiling
//================================================================================================================
void Gripper::setMinOpSpeed(int minSpeed) {
    if (minSpeed < 0) minSpeed = 0;
    else if (minSpeed > MAX_STEPPER_SPEED) minSpeed = MAX_STEPPER_SPEED;
    else minOpSpeed = minSpeed;
}

//================================================================================================================
// Sets acceleration for motion profiling
//================================================================================================================
void Gripper::setOpAccel(float accel) {
    if (accel < 0) accel = 0;
    opAccel = accel;
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

void Gripper::verbose(int *PS4_status) {
    Serial.println("================================================");
    Serial.println("GRIPPER STATUS:");
    Serial.print(" Position: ");
    Serial.println(pos);
    Serial.print(" FinalPos: ");
    Serial.println(finalPos);
    Serial.print(" Step count: ");
    Serial.println(step);
    Serial.print(" Speed: ");
    Serial.println(speed);
    Serial.print(" FSM State: ");
    if (fsmState == STATE_INIT) Serial.println("INIT");
    else if (fsmState == STATE_IDLE) Serial.println("IDLE");
    else if (fsmState == STATE_GOOFY) Serial.println("GOOFY");
    else if (fsmState == STATE_UNIFORM_MOVE) Serial.println("UNIFORM_MOVE");
    else if (fsmState == STATE_ARM) Serial.println("ARM");
    else if (fsmState == STATE_CALIBRATE_SERIAL) Serial.println("CALIBRATE_SERIAL");
    else if (fsmState == STATE_CALIBRATE_PS4) Serial.println("CALIBRATE_PS4");
    else if (fsmState == STATE_GRIP) Serial.println("GRIP");
    else if (fsmState == STATE_SORT) Serial.println("SORT");
    else if (fsmState == STATE_MOVE_BOX) Serial.println("MOVE_BOX");
    else if (fsmState == STATE_RELEASE) Serial.println("RELEASE");
    else if (fsmState == STATE_DEBUG_STEPPER) Serial.println("DEBUG_STEPPER");
    else if (fsmState == STATE_DEBUG_TRACK_PRESS) Serial.println("DEBUG_TRACK_PRESS");
    else if (fsmState == STATE_DEBUG_SERVO) Serial.println("DEBUG_SERVO");
    else if (fsmState == STATE_DEBUG_PRINT_PRESS) Serial.println("DEBUG_PRINT_PRESS");
    else Serial.println("UNKNOWN");

    // Print pressed buttons
    Serial.println(" Pressed Buttons:");
    if (PS4_status[BUTTON_CROSS] != 0) Serial.print(" X ");
    if (PS4_status[BUTTON_CIRCLE] != 0) Serial.print(" O ");
    if (PS4_status[BUTTON_SQUARE] != 0) Serial.print(" [] ");
    if (PS4_status[BUTTON_TRIANGLE] != 0) Serial.print(" Δ ");
    if (PS4_status[BUTTON_UP] != 0) Serial.print(" ^ ");
    if (PS4_status[BUTTON_DOWN] != 0) Serial.print(" v ");
    if (PS4_status[BUTTON_LEFT] != 0) Serial.print(" < ");
    if (PS4_status[BUTTON_RIGHT] != 0) Serial.print(" > ");
    Serial.println();
}









controller::controller() {
    // Initialize PID constants
    pressure = 0.0;
    pressureSetpoint = 0.0;
    error = 0.0;
    lastError = 0.0;
    integral = 0.0;
    derivative = 0.0;

    pidOutput = 0.0;
    analogOutput = 0.0;
    outputSpeed = 0;
}

void controller::PID_computer() {
    // Read current pressure
    readPressure();

    // Calculate error
    error = pressureSetpoint - pressure;

    // Integral term
    integral += error;

    // Derivative term
    derivative = error - lastError;

    // Total PID output
    pidOutput = (KP * error) + (KI * integral) + (KD * derivative);

    // Save error for next iteration
    lastError = error;

    // Convert PID output to analog output and position
    analogOutput = pidOutput * speedFactor;

    if (analogOutput > MAX_STEPPER_SPEED) {
        analogOutput = MAX_STEPPER_SPEED;
    } else if (analogOutput < 0) {
        analogOutput = 0;
    }

    outputSpeed = static_cast<int>(analogOutput);
}

bool controller::checkSteadyState() {
    static float errorSum = 0.0;
    static int errorCount = 0;
    static float average = ERROR_THRESHOLD + 1.0;
    static unsigned long lastResetTime = 0;
    static bool steady = false;

    unsigned long currentTime = millis();
    
    // Accumulate error
    errorSum += abs(error);
    errorCount++;
    
    // Check if interval has elapsed
    if (currentTime - lastResetTime >= AVERAGE_TIME_INTERVAL) {
        average = errorSum / errorCount;
        
        // Reset for next interval
        errorSum = 0.0;
        errorCount = 0;
        lastResetTime = currentTime;
        steady = (average < ERROR_THRESHOLD);
    }
        
    return steady;
}

void controller::setPressureSetpoint(float setpoint) {
    pressureSetpoint = setpoint;
}

void controller::readPressure() {
    static float analogOutput = 0.0;
    analogOutput = analogRead(PRESSURE_SENSOR_PIN);
    pressure = analogOutput;
}

int controller::getOutputSpeed() {
    return outputSpeed;
}

float controller::getPressure() {
    return pressure;
}







void PS4_cmdHandler(Gripper* gripper, int *PS4_status) {

    static bool rotating = false;
    
    if (PS4_status[CONNEXION_STATE] == 0) {
        return;
    }

    if (PS4_status[BUTTON_OPTIONS] == 1) { // Options button pressed
        gripper->setState(STATE_CALIBRATE_PS4);
        PS4_status[BUTTON_OPTIONS] = -1;
    }

    if (PS4_status[BUTTON_R1] == 1 && gripper->getfsmState() != STATE_IDLE && gripper->getfsmState() != STATE_GOOFY) { // Circle button pressed --> disarm
        if (gripper->getfsmState() != STATE_INIT) {
            gripper->setState(STATE_IDLE);
            Serial.println("Gripper disarmed.");
        }
        PS4_status[BUTTON_R1] = -1;
    }
    if (PS4_status[BUTTON_R1] == 1 && gripper->getfsmState() != STATE_ARM) { // Circle button pressed --> arm
        if (gripper->getfsmState() != STATE_INIT && gripper->getfsmState() != STATE_GOOFY) {
            gripper->setPosition(gripper->getPosition()); // hold current position
            gripper->setState(STATE_ARM);
            Serial.println("Gripper armed.");
        }
        PS4_status[BUTTON_R1] = -1;
    }

    if (PS4_status[BUTTON_UP] == 1) { // Up button pressed
        gripper->setPosition(MIN_POSITION); // Close gripper
        PS4_status[BUTTON_UP] = -1;
    }
    if (PS4_status[BUTTON_DOWN] == 1) { // Down button pressed
        gripper->setPosition(MAX_POSITION); // Open gripper
        PS4_status[BUTTON_DOWN] = -1;
    }

    if (PS4_status[BUTTON_CIRCLE] == 1) { // Circle button pressed
        if (gripper->getfsmState() == STATE_ARM) {
            gripper->setState(STATE_GRIP);
        }
        PS4_status[BUTTON_CIRCLE] = -1;
    }

    if (PS4_status[BUTTON_RIGHT] == 1) { // Right button pressed, long press enabled
        if (gripper->getfsmState() == STATE_ARM) {
            gripper->setState(STATE_UNIFORM_MOVE);
        }
        PS4_status[BUTTON_RIGHT] = -1;
    }

    if (PS4_status[BUTTON_LEFT] == 1) { // Left button pressed, long press enabled
        if (gripper->getfsmState() == STATE_ARM) {
            gripper->setState(STATE_UNIFORM_MOVE);
        }
        PS4_status[BUTTON_LEFT] = -1;
    }

    if (PS4_status[BUTTON_TRIANGLE] == 1) { // Triangle button pressed
        gripper->verbose(PS4_status);
        PS4_status[BUTTON_TRIANGLE] = -1;
    }

    if (PS4_status[BUTTON_SHARE] == 1) { // Share button pressed
        gripper->verboseEnabled = !gripper->verboseEnabled;
        PS4_status[BUTTON_SHARE] = -1;
    }
}


void commandHandler(Gripper* gripper, int *PS4_status) {

    static const char* cmdStrings[] = {
    "set",
    "get",
    "cal",
    "arm",
    "disarm",
    "grip",
    "release",
    "sort",
    "movebox",
    "status",
    "verbose",
    "debug_stepper",
    "debug_track_press",
    "debug_servo",
    "debug_print_press"
    };

    static char cmd[32], arg1[32], arg2[32];

    if (readMessage(cmd, arg1, arg2)) {
        if (strcmp(cmd, cmdStrings[0]) == 0) { // set
            if (strlen(arg1) == 0) {
                Serial.println("Error: Missing setting argument.");
                return;
            }
            if (strcmp(arg1, "fpos") == 0) { // set fpos
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing position argument.");
                    return;
                }
                int32_t newPos = atoi(arg2);
                gripper->setPosition(newPos);
                Serial.println("Final position set.");
            }
            if (strcmp(arg1, "v") == 0) { // set v
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing speed argument.");
                    return;
                }
                int newSpeed = atoi(arg2);
                gripper->setSpeed(newSpeed);
                Serial.println("Speed set.");
            }
            if (strcmp(arg1, "ms") == 0) { // set ms
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing microstepping argument.");
                    return;
                }
                int newMicroSteps = atoi(arg2);
                gripper->setMicroSteppingMode(newMicroSteps);
                Serial.println("Microstepping mode set.");
            }
            if (strcmp(arg1, "vmax") == 0) { // set vmax
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing max speed argument.");
                    return;
                }
                int maxSpeed = atoi(arg2);
                gripper->setMaxOpSpeed(maxSpeed);
                Serial.println("Max operational speed set.");
            }
            if (strcmp(arg1, "vmin") == 0) { // set vmin
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing min speed argument.");
                    return;
                }
                int minSpeed = atoi(arg2);
                gripper->setMinOpSpeed(minSpeed);
                Serial.println("Min operational speed set.");
            }
            if (strcmp(arg1, "accel") == 0) { // set accel
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing acceleration argument.");
                    return;
                }
                float accel = atof(arg2);
                gripper->setOpAccel(accel);
                Serial.println("Operational acceleration set.");
            }
        }
        if (strcmp(cmd, cmdStrings[1]) == 0) { // get
            if (strlen(arg1) == 0) {
                Serial.println("Error: Missing get argument.");
                return;
            }
            if (strcmp(arg1, "pos") == 0) { // get pos
                Serial.print("Current Position: ");
                Serial.println(gripper->getPosition());
            }
            if (strcmp(arg1, "v") == 0) { // get v
                Serial.print("Current Speed: ");
                Serial.println(gripper->getSpeed());
            }
            if (strcmp(arg1, "ms") == 0) { // get ms
                Serial.print("Current Microstepping Mode: ");
                Serial.println(gripper->getMicroSteppingMode());
            }
            if (strcmp(arg1, "fpos") == 0) { // get fpos
                Serial.print("Final Position: ");
                Serial.println(gripper->getFinalPosition());
            }
            if (strcmp(arg1, "state") == 0) { // get state
                Serial.print("Current FSM State: ");
                Serial.println(gripper->getfsmState());
            }
        }
        if (strcmp(cmd, cmdStrings[2]) == 0) { // cal
            gripper->setState(STATE_CALIBRATE_SERIAL);
        }
        if (strcmp(cmd, cmdStrings[3]) == 0) { // arm
            if (gripper->getfsmState() != STATE_INIT && gripper->getfsmState() != STATE_GOOFY) {
                gripper->setPosition(gripper->getPosition()); // hold current position
                gripper->setState(STATE_ARM);
                Serial.println("Gripper armed.");
            }
        }
        if (strcmp(cmd, cmdStrings[4]) == 0) { // disarm
            if (gripper->getfsmState() != STATE_INIT && gripper->getfsmState() != STATE_GOOFY) {
                gripper->setState(STATE_IDLE);
                Serial.println("Gripper disarmed.");
            }
        }
        if (strcmp(cmd, cmdStrings[9]) == 0) { // status
            gripper->verbose(PS4_status);
        }
        if (strcmp(cmd, cmdStrings[10]) == 0) { // verbose
            if (strlen(arg1) == 0) {
                Serial.println("Error: Missing verbose argument.");
                return;
            }
            if (strcmp(arg1, "on") == 0) { // verbose on
                gripper->verboseEnabled = true;
                Serial.println("Verbose mode enabled.");
            } else if (strcmp(arg1, "off") == 0) { // verbose off
                gripper->verboseEnabled = false;
                Serial.println("Verbose mode disabled.");
            } else {
                Serial.println("Error: Invalid verbose argument. Use 'on' or 'off'.");
            }
        }


        // Handle other commands similarly...
    }
    else return;
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
            Serial.print("Error: Command too long");
        }
    }
    
    return false; // No complete command yet
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

void limitedBeep(int ms, int bips) {           //beep temporaire
  while (bips) {
    digitalWrite(BUZZER_PIN,HIGH);
    delay(ms);
    digitalWrite(BUZZER_PIN,LOW);
    delay(ms);
    bips--;
  }
}

// Helper conversions (keep near top of file or before class methods)
static inline float rpmToStepsPerSec(int rpm) {
    return (rpm * 200.0f) / 60.0f; // full steps/sec (microstepping excluded intentionally)
}
static inline int stepsPerSecToRpm(float sps) {
    return (int) ((sps * 60.0f) / 200.0f);
}