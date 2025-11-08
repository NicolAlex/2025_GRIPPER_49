#include <MobaTools.h>
#include "constants.h"

class Gripper {
public:

void stepperMove();
bool setupGripper();
void setOrigin();
void enableStepper();
void disableStepper();
void setMicroStepping(int steppingMode); // Optional: function to set microstepping mode
void moveUntilClosed(); // Optional: function to move gripper until fully closed
int getCommand();
void testStepCmd();
void testRotateCmd();

private:
bool MS1state = LOW; // Default microstepping state for MS1
bool MS2state = LOW; // Default microstepping state for MS2
int microSteppingMode = 1; // Default microstepping mode (1x)
int speed;
int steps;


};




