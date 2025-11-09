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



void sendMessage(const char* msg);
char* readMessage();
int getCommand();