#include <MobaTools.h>
#include "constants.h"

class controller {
public:
    controller();
    //~controller();

    void PID_computer();
    
    bool checkSteadyState();

    void setPressureSetpoint(float setpoint);
    void readPressure();

    int getOutputSpeed();
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

extern controller gripController;
class Gripper {
public:
    Gripper();
    //~Gripper();

    void fsm_loop(int *PS4_status);
    
    void stepperUpdate();

    void PPM_computer();

    void stepperSetDir(int direction, int runSpeed, int microSteps);

    bool setupGripper();

    void setState(int newState);
    void setPosition(int32_t newPos);
    void setSpeed(int newSpeed);
    void setMicroSteppingMode(int newMicroSteps);
    void setMaxOpSpeed(int maxSpeed);
    void setMinOpSpeed(int minSpeed);
    void setOpAccel(float accel);

    int32_t getPosition();
    int32_t getStepCount();
    int getMicroSteppingMode();
    int getSpeed();
    int32_t getFinalPosition();
    int getfsmState();
    
    void getFruitSize();

    void stepperSetOrigin_fromSerial();
    void stepperSetOrigin_fromPS4(int *PS4_status);

    void stepperEnable();
    void stepperDisable();

    void servoSetup();
    bool servoRotate();

    void verbose(int *PS4_status);

    // public variables
    bool verboseEnabled;

private:
    inline void setMicroSteps();
    void stepperMoveSteps(int steps, int runSpeed);

    int fsmState;

    bool MS1state; // Default microstepping state for MS1
    bool MS2state; // Default microstepping state for MS2
    bool enabled;
    int microSteppingMode; // Default microstepping mode (2x)
    int speed;
    float pos;
    float finalPos;
    int32_t lastStep;
    int32_t step;

    int maxOpSpeed;
    int minOpSpeed;
    float opAccel;

    int fruitSize;


};

void commandHandler(Gripper* gripper);
void PS4_cmdHandler(int *PS4_status, Gripper* gripper);

float InterpolToAngle(float length);
float InterpolToLength(float angle);

void sendMessage(const char* msg);
bool readMessage(char* outCmd, char* outArg1, char* outArg2);

void statusLedBlinking(bool enabled);
void buzzerBeep(int duration, bool setup); // if setup is true, start the beep, if false, stop after duration
void limitedBeep(int ms, int bips); // blocking beep for duration ms