#include <MobaTools.h>
#include "constants.h"

class Gripper {
public:
    Gripper();
    //~Gripper();

    void fsm_loop();
    
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

    void stepperSetOrigin();

    void stepperEnable();
    void stepperDisable();

    void servoSetup();
    bool servoRotate();

    void verbose();

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
    int pos;
    int finalPos;
    int lastStep;
    int step;

    int maxOpSpeed;
    int minOpSpeed;
    float opAccel;

    int fruitSize;


};

void commandHandler(Gripper* gripper);

float InterpolToAngle(float length);
float InterpolToLength(float angle);

void sendMessage(const char* msg);
bool readMessage(char* outCmd, char* outArg1, char* outArg2);

void statusLedBlinking(bool enabled);
void buzzerBeep(int duration, bool setup); // if setup is true, start the beep, if false, stop after duration