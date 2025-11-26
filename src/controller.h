#include "constants.h"

class controller {
public:
    controller();
    //~controller();

    void PID_computer();

    void setPressureSetpoint(float setpoint);
    void readPressure();

    float getOuputSpeed();
    float getPressure();
    float getPressureSetpoint();

private:
    const float KP = 0.0;
    const float KI = 0.0;
    const float KD = 0.0;

    const float speedFactor = 1.0;

    float pressure;
    float pressureSetpoint;
    float error;
    float lastError;
    float integral;
    float derivative;

    float pidOutput;
    float analogOutput;
    int outputSpeed;

};