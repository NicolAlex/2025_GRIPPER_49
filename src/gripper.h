#include <MobaTools.h>
#include "constants.h"

class Gripper {
public:

void stepperUpdate();

void stepperSetDir(int direction, int runSpeed, int microSteps);

bool setupGripper();

int getPosition();
int getStepCount();

void stepperSetOrigin();

void stepperEnable();
void stepperDisable();

private:
void setMicroSteps();
void stepperMoveSteps(int steps, int runSpeed);

bool MS1state = LOW; // Default microstepping state for MS1
bool MS2state = LOW; // Default microstepping state for MS2
bool enabled = false;
int microSteppingMode = 1; // Default microstepping mode (1x)
int speed = 0;
int pos = 0;
int finalPos = 0;
int lastStep = 0;
int step = 0;


};




