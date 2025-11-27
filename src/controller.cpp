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
    pressure = (analogOutput / 4095.0f) * 100.0f; // Example conversion to pressure in kPa
}