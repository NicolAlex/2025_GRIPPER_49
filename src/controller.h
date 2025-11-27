
class controller {
public:
    controller();
    //~controller();

    void PID_computer();
    
    bool checkSteadyState();

    void setPressureSetpoint(float setpoint);
    void readPressure();

    float getOuputSpeed();
    float getPressure();
    float getSetpoint();

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