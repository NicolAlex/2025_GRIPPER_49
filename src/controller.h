#include "constants.h"

class controller {
public:
    controller();
    //~controller();

    void PID_computer();
    
    int convertToSteps();

    bool checkSteadyState();

    void setPressureSetpoint(float setpoint);
    void readPressure();

    float getOuputOpening();
    float getPressure();
    float getSetpoint();

private:
    const float KP = 0.0;
    const float KI = 0.0;
    const float KD = 0.0;

    const float factor = 1.0;

    float pressure;
    float pressureSetpoint;
    float error;
    float lastError;
    float integral;
    float derivative;

    float pidOutput;
    float analogOutput;
    int outputOpening;

};