#include <MobaTools.h>
#include "constants.h"

class Gripper {
public:
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

private:
void setMicroSteps();
void stepperMoveSteps(int steps, int runSpeed);

bool MS1state = LOW; // Default microstepping state for MS1
bool MS2state = LOW; // Default microstepping state for MS2
bool enabled = false;
int microSteppingMode = 2; // Default microstepping mode (2x)
int speed = 100;
int pos = 0;
int finalPos = 1000;
int lastStep = 99;
int step = 0;


};



void sendMessage(const char* msg);
char* readMessage();
int getCommand();