#include "Arduino.h"
#include "controller.h"


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
    error = pressureSetpoint - pressure;
    integral += error;
    derivative = error - lastError;

    pidOutput = KP * error + KI * integral + KD * derivative;

    analogOutput = pidOutput * speedFactor;

    if (analogOutput > MAX_STEPPER_SPEED) {
        analogOutput = MAX_STEPPER_SPEED;
    }
    else if (analogOutput < -MAX_STEPPER_SPEED) {
        analogOutput = -MAX_STEPPER_SPEED;
    }

    outputSpeed = static_cast<int>(analogOutput);

    lastError = error;
}

void controller::setPressureSetpoint(float setpoint) {
    pressureSetpoint = setpoint;
}