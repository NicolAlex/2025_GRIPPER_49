#include <Arduino.h>
#include <MobaTools.h>
#include "gripper.h"

MoToStepper gripperStepper(200, STEPDIR); // 200 steps per revolution, STEPDIR mode
MoToServo boxServo;

controller gripController;

// convertion functions def
static inline float rpmToStepsPerSec(int rpm);
static inline int stepsPerSecToRpm(float stepsPerSec);


//================================================================================================================
// Constructor: Initialize gripper with default values
//================================================================================================================
Gripper::Gripper() {
    MS1state = HIGH; // Default microstepping state for MS1
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
    releaseTimer = 5000;
    fruitSize = UNDEFINED_SIZE;
    verboseEnabled = false;
    ripe = true;
    fruitSizeThreshold = 1000.0f; // default size threshold

}


//================================================================================================================
// FSM loop: Handles state machine transitions and state-specific behaviors
//================================================================================================================
void Gripper::fsm_loop(int *PS4_status) {
    static unsigned long startTime_controller = millis();
    static unsigned long startTime_release = millis();
    switch(fsmState) {
        case STATE_INIT:
            setupGripper();
            boxServo.attach(SERVO_PIN);
            servoWobble(BOX_HOLD); // initial position
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
            gripController.setPressureOffset();
            fsmState = STATE_ARM;
            break;
        case STATE_CALIBRATE_PS4:
            Serial.println("Starting calibration...");
            stepperSetOrigin_fromPS4(PS4_status);
            gripController.setPressureOffset();
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
            statusLedBlinking(LOW);
            break;
        case STATE_FEEL:
            if (!ripe) {
                limitedBeep(25, 12); // alert for unripe fruit
                Serial.println("Fruit not ripe enough to grip.");
                fsmState = STATE_ARM;
                finalPos = MAX_POSITION - 1;
                PPM_computer();
                finalPos = MAX_POSITION;
            }
            else {
                ripe = true;
                gripController.readPressure();
                if (gripController.checkSize()) {
                    finalPos = pos; // hold position
                    fsmState = STATE_SORT;
                    statusLedBlinking(LOW);
                }
                boxServo.write(BOX_HOLD); // hold position
                PPM_computer();
                statusLedBlinking(HIGH);
            }
            break;
        case STATE_GRIP:
            // Grip state actions
            static int gripSpeed = 0;
            gripController.readPressure();
            gripSpeed = gripController.getOutputSpeed();
            if (millis() - startTime_controller > 150 && gripController.checkSteadyState()) {
                limitedBeep(10, 20); // confirm grip
                startTime_release = millis();
                fsmState = STATE_RELEASE;
                finalPos = pos;
                speed = 0;
                statusLedBlinking(LOW);
                break;
            }
            //Serial.print(">press:");
            //Serial.println(gripController.getPressure(), 3);
            //microSteppingMode = 16; // best precision for gripping
            gripController.PID_computer();
            stepperSetDir( (gripSpeed >=0) ? 1 : -1, abs(gripSpeed), microSteppingMode);
            statusLedBlinking(HIGH);
            break;
        case STATE_SORT:
            // Sort state actions
            microSteppingMode = 2; // default microstepping mode
            Serial.print("Size of fruit: ");
            Serial.print(pos);
            Serial.println(" steps");
            Serial.print("Classified as: ");
            if (pos < fruitSizeThreshold) {
                fruitSize = FRUIT_SMALL;
                Serial.println("SMALL");
                Serial.println("----------------");
                limitedBeep(200, 2);
            } else {
                fruitSize = FRUIT_LARGE;
                Serial.println("LARGE");
                Serial.println("----------------");

                limitedBeep(100, 4);
            }
            fsmState = STATE_MOVE_BOX;
            break;
        case STATE_MOVE_BOX:
            boxServo.write(fruitSize == FRUIT_SMALL ? BOX_SMALL : BOX_LARGE); // hold position
            fsmState = STATE_GRIP;
            startTime_controller = millis();
            break;
        case STATE_RELEASE:
            if (PS4_status[BUTTON_CROSS] == 1 || millis() - startTime_release > 5000) { // Circle button pressed or timeout
                finalPos = MAX_POSITION - 1;
                PPM_computer();
                finalPos = MAX_POSITION;
                fsmState = STATE_ARM;
                PS4_status[BUTTON_CROSS] = -1;
            }
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
    step = -gripperStepper.readSteps(); // Negate to match inverted direction
    
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
    gripperStepper.doSteps(-steps * microSteppingMode); // Negated to invert direction
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

//================================================================================================================
// Interactive calibration from PS4 controller: waits for user to manually close gripper
// Waits for PS4 O button press to confirm, then sets position to zero
//================================================================================================================
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
            MS1state = HIGH;
            MS2state = LOW;
            break;
    }
    digitalWrite(MS1_PIN, MS1state);
    digitalWrite(MS2_PIN, MS2state);
}

//================================================================================================================
// GRIPPER GETTERS: Return current gripper state and position information
//================================================================================================================
int32_t Gripper::getPosition() {
    return pos;
}

int32_t Gripper::getFinalPosition() {
    return finalPos;
}

int32_t Gripper::getStepCount() {
    return step;
}

int32_t Gripper::getMicroSteppingMode() {
    return microSteppingMode;
}

int Gripper::getSpeed() {
    return speed;
}

int Gripper::getfsmState() {
    return fsmState;
}

//================================================================================================================
// GRIPPER SETTERS: Configure gripper behavior and parameters
//================================================================================================================
void Gripper::setPosition(int32_t newPos) {
    finalPos = newPos;
}

void Gripper::setSpeed(int newSpeed) {
    speed = newSpeed;
}

void Gripper::setMicroSteppingMode(int newMicroSteps) {
    if (newMicroSteps != 2 && newMicroSteps != 4 && newMicroSteps != 8 && newMicroSteps != 16) {
        Serial.println("Invalid microstepping mode. Must be 2, 4, 8, or 16.");
        microSteppingMode = 2;
        return;
    }
    microSteppingMode = newMicroSteps;
}

void Gripper::setMaxOpSpeed(int maxSpeed) {
    if (maxSpeed < 0) maxSpeed = 0;
    else if (maxSpeed > MAX_STEPPER_SPEED) maxSpeed = MAX_STEPPER_SPEED;
    else maxOpSpeed = maxSpeed;
}

void Gripper::setMinOpSpeed(int minSpeed) {
    if (minSpeed < 0) minSpeed = 0;
    else if (minSpeed > MAX_STEPPER_SPEED) minSpeed = MAX_STEPPER_SPEED;
    else minOpSpeed = minSpeed;
}

void Gripper::setOpAccel(float accel) {
    if (accel < 0) accel = 0;
    opAccel = accel;
}

void Gripper::setRipeness(bool isRipe) {
    ripe = isRipe;
}

void Gripper::setFruitSizeThreshold(float threshold) {
    fruitSizeThreshold = threshold;
}

void Gripper::setReleaseTimer(unsigned long duration) {
    releaseTimer = duration;
}

//================================================================================================================
// Makes the servo wobble between two positions
// Animates servo movement to show visual feedback or reset position
//================================================================================================================
void Gripper::servoWobble(int angle) {
    static unsigned long lastTime = 0;
    if (angle > servoLastAngle) {
        lastTime = millis();
        while(millis() - lastTime < 3000) {
            boxServo.write(BOX_SMALL_EMPTY);
            delay(25);
            boxServo.write(angle);
            delay(100);
        }
        servoLastAngle = angle;
    }
    else {
        lastTime = millis();
        while(millis() - lastTime < 3000) {
            boxServo.write(BOX_LARGE_EMPTY);
            delay(50);
            boxServo.write(angle);
            delay(150);
        }
        servoLastAngle = angle;
    }
}

//================================================================================================================
// Prints detailed gripper status information to Serial
// Displays current position, speed, FSM state, controller error, and pressed buttons
//================================================================================================================
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
    Serial.print(" controller error :");
    Serial.println(gripController.getError());
    Serial.print(" FSM State: ");
    if (fsmState == STATE_INIT) Serial.println("INIT");
    else if (fsmState == STATE_IDLE) Serial.println("IDLE");
    else if (fsmState == STATE_GOOFY) Serial.println("GOOFY");
    else if (fsmState == STATE_UNIFORM_MOVE) Serial.println("UNIFORM_MOVE");
    else if (fsmState == STATE_ARM) Serial.println("ARM");
    else if (fsmState == STATE_CALIBRATE_SERIAL) Serial.println("CALIBRATE_SERIAL");
    else if (fsmState == STATE_CALIBRATE_PS4) Serial.println("CALIBRATE_PS4");
    else if (fsmState == STATE_GRIP) Serial.println("GRIP");
    else if (fsmState == STATE_FEEL) Serial.println("FEEL");
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

//================================================================================================================
// Constructor: Initialize controller with default PID parameters and values
//================================================================================================================
controller::controller() {
    // Initialize PID
    KP = 1;
    KI = 0.0;
    KD = 0.1;
    pressure = 0.0;
    pressureSetpoint = 1200.0; // default setpoint
    pressureOffset = 0.0;
    pressureSizeThreshold = 50.0f; // default size threshold
    error = 0.0;
    lastError = 0.0;
    integral = 0.0;
    derivative = 0.0;

    pidOutput = 0.0;
    analogOutput = 0.0;
    outputSpeed = 0;
}

//================================================================================================================
// Computes PID control output for pressure regulation
// Calculates proportional, integral, and derivative terms based on error and converts to speed command
//================================================================================================================
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
    analogOutput = - (pidOutput * speedFactor);

    if (analogOutput > MAX_SPEED_CONTROL) {
        analogOutput = MAX_SPEED_CONTROL;
    } else if (analogOutput < - MAX_SPEED_CONTROL) {
        analogOutput = - MAX_SPEED_CONTROL;
    }

    outputSpeed = static_cast<int>(analogOutput);
}

//================================================================================================================
// Checks if fruit size threshold has been reached
// Accumulates average pressure over time interval and compares against size threshold
// Returns true if average pressure exceeds threshold within the sampling interval
//================================================================================================================
bool controller::checkSize() {
    static float pressureSum = 0.0f;
    static int pressureCount = 0;
    static unsigned long lastResetTime = 0;
    static float averagePressure = 0.0f;

    unsigned long currentTime = millis();

    // Accumulate pressure readings
    pressureSum += pressure;
    pressureCount++;

    // Check if interval has elapsed
    if (currentTime - lastResetTime >= PRESSURE_AVERAGE_TIME_INTERVAL) {
        averagePressure = pressureSum / pressureCount;
        
        // Reset for next interval
        pressureSum = 0.0f;
        pressureCount = 0;
        lastResetTime = currentTime;
    }

    // Return true if average pressure exceeds the threshold
    return (averagePressure >= pressureSizeThreshold);
}

//================================================================================================================
// Checks if the controller error has reached a steady state
// Averages absolute error over time interval and compares against error threshold
// Returns true if average error is below threshold (steady state achieved)
//================================================================================================================
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
    if (currentTime - lastResetTime >= ERROR_AVERAGE_TIME_INTERVAL) {
        average = errorSum / errorCount;
        
        // Reset for next interval
        errorSum = 0.0;
        errorCount = 0;
        lastResetTime = currentTime;
        steady = (average < ERROR_THRESHOLD);
    }
        
    return steady;
}

//================================================================================================================
// CONTROLLER SETTERS: Configure pressure control parameters
//================================================================================================================
void controller::setPressureSetpoint(float setpoint) {
    pressureSetpoint = setpoint;
}

void controller::setPressureOffset() {
    static float analogOutput = 0.0;
    analogOutput = analogRead(PRESSURE_SENSOR_PIN);
    pressureOffset = analogOutput;
}

void controller::setPressureSizeThreshold(float threshold) {
    pressureSizeThreshold = threshold;
}

void controller::setKP(float kp) {
    KP = kp;
}

void controller::setKI(float ki) {
    KI = ki;
}

void controller::setKD(float kd) {
    KD = kd;
}

void controller::readPressure() {
    static float analogOutput = 0.0;
    analogOutput = analogRead(PRESSURE_SENSOR_PIN);
    pressure = analogOutput - pressureOffset; // apply offset
}

//================================================================================================================
// CONTROLLER GETTERS: Return controller state and sensor readings
//================================================================================================================
int controller::getOutputSpeed() {
    return outputSpeed;
}

float controller::getPressure() {
    return pressure;
}

float controller::getError() {
    return error;
}

//================================================================================================================
// Handles PS4 controller input commands and updates gripper FSM state
// Maps button presses to gripper control actions (calibration, arm/disarm, movement, etc.)
//================================================================================================================
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
            gripper->setState(STATE_FEEL);
            gripper->setPosition(MIN_POSITION); // close gripper
            gripController.setPressureOffset();
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

    if (PS4_status[BUTTON_R2] == 1) { // R2 button pressed
        boxServo.write(BOX_SMALL_EMPTY); // empty small box
        limitedBeep(100, 1);
        PS4_status[BUTTON_R2] = -1;
    }

    if (PS4_status[BUTTON_L2] == 1) { // L2 button pressed
        gripper->servoWobble(BOX_LARGE_EMPTY); // empty large box
        limitedBeep(100, 1);
        PS4_status[BUTTON_L2] = -1;
    }

    if (PS4_status[BUTTON_L1] == 1) { // L1 button pressed
        gripper->servoWobble(BOX_HOLD); // hold position
        limitedBeep(100, 1);
        PS4_status[BUTTON_L1] = -1;
    }
}

//================================================================================================================
// Handles serial command input and routes to appropriate gripper control functions
// Parses commands (set, get, cal, arm, disarm, etc.) and updates gripper parameters
//================================================================================================================
void serial_cmdHandler(Gripper* gripper, int *PS4_status) {

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
    "debug_print_press",
    "RIPE",
    "UNRIPE"
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
            if (strcmp(arg1, "kp") == 0) { // set kp
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing KP argument.");
                    return;
                }
                float kp = atof(arg2);
                gripController.setKP(kp);
                Serial.println("Controller KP set.");
            }
            if (strcmp(arg1, "ki") == 0) { // set ki
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing KI argument.");
                    return;
                }
                float ki = atof(arg2);
                gripController.setKI(ki);
                Serial.println("Controller KI set.");
            }
            if (strcmp(arg1, "kd") == 0) { // set kd
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing KD argument.");
                    return;
                }
                float kd = atof(arg2);
                gripController.setKD(kd);
                Serial.println("Controller KD set.");
            }
            if (strcmp(arg1, "pset") == 0) { // set pset
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing pressure setpoint argument.");
                    return;
                }
                float pset = atof(arg2);
                gripController.setPressureSetpoint(pset);
                Serial.println("Pressure setpoint set.");
            }
            if (strcmp(arg1, "servo") == 0) { // set servo
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing servo position argument.");
                    return;
                }
                int servoPos = atoi(arg2);
                gripper->servoWobble(servoPos);
                Serial.println("Servo position set.");
            }
            if (strcmp(arg1, "psize") == 0) { // set pressure size threshold
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing pressure size threshold argument.");
                    return;
                }
                float psize = atof(arg2);
                gripController.setPressureSizeThreshold(psize);
                Serial.println("Pressure sensor offset calibrated.");
            }
            if (strcmp(arg1, "pthr") == 0) { // set pressure grip threshold
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing pressure size threshold argument.");
                    return;
                }
                float pthr = atof(arg2);
                gripController.setPressureSizeThreshold(pthr);
                Serial.println("Pressure size threshold set.");
            }
            if (strcmp(arg1, "sthr") == 0) { // set size threshold
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing fruit size threshold argument.");
                    return;
                }
                float sthr = atof(arg2);
                gripper->setFruitSizeThreshold(sthr);
                Serial.println("Fruit size threshold set.");
            }
            if (strcmp(arg1, "rtimer") == 0) { // set release timer
                if (strlen(arg2) == 0) {
                    Serial.println("Error: Missing release timer argument.");
                    return;
                }
                unsigned long rtimer = atol(arg2);
                gripper->setReleaseTimer(rtimer);
                Serial.println("Release timer duration set.");
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
        if (strcmp(cmd, cmdStrings[15]) == 0) { // RIPE information
            gripper->setRipeness(true);
            digitalWrite(LED5_PIN, HIGH); // Indicate ripe status
        }
        if (strcmp(cmd, cmdStrings[16]) == 0) { // UNRIPE information
            if (gripper->getfsmState() != STATE_FEEL) {
                gripper->setRipeness(false);
                digitalWrite(LED5_PIN, LOW); // Indicate unripe status
            }
        }


        // Handle other commands similarly...
    }
    else return;
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
// Produces repeated buzzer beeps: blocking function that produces ms-duration beeps separated by ms delays
// bips parameter controls number of beep pulses to emit
//================================================================================================================
void limitedBeep(int ms, int bips) {
  while (bips) {
    digitalWrite(BUZZER_PIN,HIGH);
    delay(ms);
    digitalWrite(BUZZER_PIN,LOW);
    delay(ms);
    bips--;
  }
}

//================================================================================================================
// Converts RPM speed to steps per second (full steps, excluding microstepping)
//================================================================================================================
static inline float rpmToStepsPerSec(int rpm) {
    return (rpm * 200.0f) / 60.0f; // full steps/sec (microstepping excluded intentionally)
}

//================================================================================================================
// Converts steps per second to RPM speed (full steps, excluding microstepping)
//================================================================================================================
static inline int stepsPerSecToRpm(float sps) {
    return (int) ((sps * 60.0f) / 200.0f);
}