#define DIR_PIN 14
#define STEP_PIN 27
#define ENABLE_PIN 26
#define MS1_PIN 33
#define MS2_PIN 25

enum COMMANDS {
    CMD_SETUP_STEPPER,
    CMD_SET_ORIGIN,
    CMD_SET_MICROSTEPPING,
    CMD_MOVE_UNTIL_CLOSED,
    CMD_ENABLE_STEPPER,
    CMD_DISABLE_STEPPER,
    CMD_DO_STEPS,
    CMD_ROTATE
};

#define INTERPOL_SAMPLES 5

const float interpolSample[5][2] = {
    {0.0, 0.0},
    {10.0, 90.0},
    {20.0, 180.0},
    {30.0, 270.0},
    {40.0, 360.0}
};



#define MAX_STEP_DIR 10000

