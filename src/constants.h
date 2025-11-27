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

// Absolute maximum stepper speed and acceleration
#define MAX_STEPPER_SPEED 4200 // steps/min * 10

// constants for PPM driver
#define ACCEL_STEPS 400      // Number of steps for acceleration/deceleration

// constants for PID controller
#define MAX_OPENING 100       // Maximum opening distance in mm
#define ERROR_THRESHOLD 0.5   // Threshold for steady state detection
#define AVERAGE_TIME_INTERVAL 100 // Time interval for averaging error in ms

//fuit size thresholds in mm
#define FRUIT_SIZE_THRESHOLD 60.0f

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

enum FSM_STATES {
    STATE_INIT,
    STATE_CALIBRATE,
    STATE_IDLE,
    STATE_ARM,
    STATE_GRIP,
    STATE_SORT,
    STATE_MOVE_BOX,
    STATE_RELEASE,
    STATE_DEBUG,

    STATE_STEPPER_SPEED_TEST
};

enum FRUIT_SIZES {
    FRUIT_SMALL,
    FRUIT_LARGE,
    UNDEFINED_SIZE
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

