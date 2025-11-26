#include <MobaTools.h>
#include "constants.h"

class Gripper {
public:
    Gripper();
    //~Gripper();
    
    void stepperUpdate();

    void stepperSetDir(int direction, int runSpeed, int microSteps);

    bool setupGripper();

    void setPosition(int32_t newPos);
    void setSpeed(int newSpeed);
    void setMicroSteppingMode(int newMicroSteps);

    int32_t getPosition();
    int32_t getStepCount();
    int getMicroSteppingMode();
    int getSpeed();
    int32_t getFinalPosition();

    void stepperSetOrigin();

    void stepperEnable();
    void stepperDisable();

    float readPressure();

    float interpolToAngle(float length);
    float interpolToLength(float angle);

private:
    inline void setMicroSteps();
    void stepperMoveSteps(int steps, int runSpeed);

    bool MS1state; // Default microstepping state for MS1
    bool MS2state; // Default microstepping state for MS2
    bool enabled;
    int microSteppingMode; // Default microstepping mode (2x)
    int speed;
    int pos;
    int finalPos;
    int lastStep;
    int step;


};


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
    const float KP;
    const float KI;
    const float KD;

    float pressure;
    float pressureSetpoint;
    float error;
    float lastError;
    float integral;
    float derivative;

    float analogOutput;
    int outputSpeed;

};


void debugCommandHandler(int cmd, Gripper* gripper);
void sendMessage(const char* msg);
char* readMessage();
int getCommand();

void statusLedBlinking(bool enabled);
void buzzerBeep(int duration, bool setup); // if setup is true, start the beep, if false, stop after duration