#include "Arduino.h"
#include "controller.h"
#include "gripper.h"


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
    outputOpening = 0;
}

void controller::PID_computer() {
    // Read current pressure
    readPressure();

    // Calculate error
    error = pressureSetpoint - pressure;

    // Proportional term
    float Pout = KP * error;

    // Integral term
    integral += error;
    float Iout = KI * integral;

    // Derivative term
    derivative = error - lastError;
    float Dout = KD * derivative;

    // Total PID output
    pidOutput = Pout + Iout + Dout;

    // Save error for next iteration
    lastError = error;

    // Convert PID output to analog output and position
    analogOutput = pidOutput * factor;

    if (analogOutput > MAX_OPENING) {
        analogOutput = MAX_OPENING;
    } else if (analogOutput < 0) {
        analogOutput = 0;
    }

    outputOpening = static_cast<int>(analogOutput);
}


int controller::convertToSteps() {
    return static_cast<int>(InterpolToAngle(outputOpening));
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