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

// Absolute maximum stepper speed
#define MAX_STEPPER_SPEED 420 // rotations/min (200 steps/rev -> 420*200=84000 steps/min)

// Absolute maximum position range
#define MAX_POSITION 100000 // in steps
#define MIN_POSITION 0      // in steps

// constants for PID controller
#define MAX_OPENING 100       // Maximum opening distance in mm
#define ERROR_THRESHOLD 0.5   // Threshold for steady state detection
#define AVERAGE_TIME_INTERVAL 100 // Time interval for averaging error in ms

//fuit size thresholds in mm
#define FRUIT_SIZE_THRESHOLD 60.0f







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

#define INTERPOL_SAMPLES 5

const float interpolSample[5][2] = {
    {0.0, 0.0},
    {10.0, 90.0},
    {20.0, 180.0},
    {30.0, 270.0},
    {40.0, 360.0}
};



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