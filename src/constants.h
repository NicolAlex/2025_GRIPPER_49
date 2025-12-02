#define DIR_PIN 14
#define STEP_PIN 27
#define ENABLE_PIN 26
#define MS1_PIN 33
#define MS2_PIN 25
#define LED1_PIN 21
#define LED2_PIN 19
#define LED3_PIN 18
#define LED4_PIN 5
#define LED5_PIN 17
#define LED6_PIN 16
#define BUZZER_PIN 32
#define PRESSURE_SENSOR_PIN 34
#define SERVO_PIN 4

// #define GET_MAC_ADDRESS

// Absolute maximum stepper speed
#define MAX_STEPPER_SPEED 420 // rotations/min (200 steps/rev -> 420*200=84000 steps/min)
#define MAX_SPEED_CONTROL 300

// Absolute maximum position range
#define MAX_POSITION 2290 // in steps
#define MIN_POSITION 0      // in steps

// constants for PID controller
#define MAX_OPENING 100       // Maximum opening distance in mm
#define ERROR_THRESHOLD 250   // Threshold for steady state detection
#define AVERAGE_TIME_INTERVAL 100 // Time interval for averaging error in ms

//fuit size thresholds in mm
#define FRUIT_SIZE_THRESHOLD 15.0f




//extern const char* cmdStrings[];  // Declaration only

enum FSM_STATES {
    STATE_INIT,
    STATE_GOOFY,
    STATE_CALIBRATE_SERIAL,
    STATE_CALIBRATE_PS4,
    STATE_IDLE,
    STATE_ARM,
    STATE_UNIFORM_MOVE,
    STATE_GRIP,
    STATE_SORT,
    STATE_MOVE_BOX,
    STATE_RELEASE,
    STATE_DEBUG_STEPPER,
    STATE_DEBUG_TRACK_PRESS,
    STATE_DEBUG_SERVO,
    STATE_DEBUG_PRINT_PRESS,
};



enum FRUIT_SIZES {
    FRUIT_SMALL,
    FRUIT_LARGE,
    UNDEFINED_SIZE
};





enum PS4_BUTTONS {
    CONNEXION_STATE,
    BUTTON_CROSS,
    BUTTON_CIRCLE,
    BUTTON_TRIANGLE,
    BUTTON_SQUARE,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_L1,
    BUTTON_R1,
    BUTTON_L2,
    BUTTON_R2,
    BUTTON_OPTIONS,
    BUTTON_SHARE,
};



enum RGB {
    R,
    G,
    B
};

const int ledArray[6] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN, LED6_PIN};

#define INTERPOL_SAMPLES 17

/*
const float interpolSample[36][2] = {
    {3, 0},
    {6, 135},
    {7, 187},
    {7.5, 231.5},
    {8.5, 275},
    {9.5, 312},
    {10, 351},
    {10.5, 391},
    {12, 419},
    {12.5, 487.5},
    {14, 529},
    {14.5, 570},
    {15.5, 601.5},
    {16, 649},
    {17, 683},
    {18, 720},
    {18.5, 762},
    {20, 805},
    {20.5, 843.5},
    {22, 908},
    {23, 944.5},
    {24, 988},
    {24.5, 1019},
    {26, 1076},
    {27, 1136},
    {28, 1175},
    {29.5, 1213},
    {31, 1295},
    {33, 1402},
    {35, 1487},
    {37.5, 1594},
    {39.5, 1689},
    {44, 1870},
    {45, 1922},
    {47, 2003},
    {51, 2290}
};
*/
const float interpolSample[17][2] = {
    {5.5, 0.0},
    {6.5, 43.0},
    {7.0, 86.5},
    {9.0, 191.5},
    {9.5, 204.5},
    {11.0, 299.0},
    {13.5, 425.0},
    {16.5, 578.5},
    {19.5, 709.5},
    {22.5, 862.0},
    {25.5, 999.5},
    {28.0, 1147.0},
    {33.0, 1340.5},
    {37.0, 1543.0},
    {42.0, 1768.5},
    {47.0, 2006.5},
    {50.0, 2290.5}
};

/*
const float interpolSample[18][2] = {
    {3.5, 0},
    {4.5, 238.5},
    {6.5, 317},
    {8.5, 400.5},
    {10, 483.5},
    {12, 566.5},
    {13, 651.5},
    {15, 736},
    {17, 821},
    {19, 904},
    {20.5, 979},
    {23, 1085},
    {26, 1227.5},
    {30, 1374.5},
    {34.5, 1605.5},
    {41, 1838.5},
    {49.5, 2266},
    {52, 2290}
};
*/



#define MAX_STEP_DIR 10000



// Command quick handbook :
// set fpos <value>         : set gripper position in steps
// set v <value>            : set gripper speed in rotations per minute
// set ms <value>           : set microstepping mode (2, 4, 8, or 16)
// set vmax <value>         : set maximum operating speed in rotations per minute
// set vmin <value>         : set minimum operating speed in rotations per minute
// set accel <value>        : set operating acceleration in rotations per minute squared

// get pos                  : get current gripper position in steps
// get v                    : get current gripper speed in rotations per minute
// get ms                   : get current microstepping mode
// get fpos                 : get final gripper position in steps
// get state                : get current FSM state

// cal                      : calibrate gripper using serial interface
// arm                      : arm the gripper (hold current position)
// disarm                   : disarm the gripper (idle state)
// status                   : print current gripper status

// verbose <on/off>         : enable/disable verbose mode (periodic status printout)

// Button mappings:
// R1 button                : arm/disarm gripper
// Up button                : close gripper
// Down button              : open gripper
// Circle button            : grip action
// Right button             : uniform move (open gripper slowly)
// Left button              : uniform move (close gripper slowly)
// Triangle button          : print status
// Share button             : toggle verbose mode
// Options button           : calibrate gripper using PS4 controller