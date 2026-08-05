//==============================================================
// Submarine Controller v2
// ESP32-S3 Hardware Controller
//
// Responsibilities
//  - Interface with Raspberry Pi
//  - Drive servos
//  - Drive thruster
//  - Drive ballast
//  - Read sensors
//  - Report telemetry
//
// The Raspberry Pi remains the "brain".
// The ESP is purely the hardware interface.
//==============================================================

#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>

//
//==============================================================
// Pi UART
//==============================================================
//

#define USE_PI_UART           1

#define PI_UART_RX            18
#define PI_UART_TX            17

#if USE_PI_UART
HardwareSerial PiLink(1);
#define LINK PiLink
#else
#define LINK Serial
#endif

//
//==============================================================
// I2C
//==============================================================
//

#define PIN_I2C_SDA           8
#define PIN_I2C_SCL           9

#define PCA9685_I2C_ADDR      0x40
#define IMU_I2C_ADDR          0x68

//
//==============================================================
// Servo Channels
//==============================================================
//

#define CH_AFT_STEER_Y        0
#define CH_AFT_STEER_Z        1
#define CH_FIN_LEFT           2
#define CH_FIN_RIGHT          3

//
//==============================================================
// Thruster
//==============================================================
//

#define PIN_THRUSTER_IN1      4
#define PIN_THRUSTER_IN2      5
#define PIN_THRUSTER_PWM      6

#define THRUSTER_PWM_FREQ     5000
#define THRUSTER_PWM_RES      8
#define THRUSTER_MAX_PWM      255
#define THRUSTER_MIN_START    90
#define THRUSTER_DEADBAND     0.02f

//
//==============================================================
// Ballast
//==============================================================
//

#define PIN_BALLAST_FILL      10
#define PIN_BALLAST_DRAIN     11
#define PIN_BALLAST_LEVEL_ADC 2

//
//==============================================================
// Battery
//==============================================================
//

#define PIN_BATTERY_ADC       1

#define BATTERY_ADC_VREF      3.3f
#define BATTERY_DIVIDER_RATIO 11.0f

//
//==============================================================
// Leak Sensors
//==============================================================
//

#define PIN_LEAK_1 12
#define PIN_LEAK_2 13
#define PIN_LEAK_3 14
#define PIN_LEAK_4 15

static const int LEAK_PINS[] =
{
    PIN_LEAK_1,
    PIN_LEAK_2,
    PIN_LEAK_3,
    PIN_LEAK_4
};

const int NUM_LEAKS =
sizeof(LEAK_PINS) /
sizeof(LEAK_PINS[0]);

//
//==============================================================
// Depth
//==============================================================
//

#define PIN_DEPTH_ADC      3
#define DEPTH_ADC_MAX_M    10.0f
#define DEPTH_I2C_ADDR     0

//
//==============================================================
// Timing
//==============================================================
//

#define SERIAL_BAUD        115200

#define SERIAL_TIMEOUT_MS  8000

#define TELEMETRY_MS       200

#define STATUS_MS          1000

//
//==============================================================
// Servo Limits
//==============================================================
//

#define SERVO_MIN          205
#define SERVO_CENTER       307
#define SERVO_MAX          410

#define PCA9685_OSC_FREQ   25000000

//
//==============================================================
// Controller State
//==============================================================
//

struct ControlState
{
    float aftY = 0.0f;

    float aftZ = 0.0f;

    float finLeft = 0.0f;

    float finRight = 0.0f;

    float thrusterCmd = 0.0f;

    int thrusterPWM = 0;

    float ballastCmd = 0.0f;
};

//
//==============================================================
// Telemetry State
//==============================================================
//

struct TelemetryState
{
    float battery = 0.0f;

    float pitch = 0.0f;

    float roll = 0.0f;

    float yaw = 0.0f;

    float depth = 0.0f;

    float ballast = 0.0f;

    uint16_t ballastADC = 0;

    bool ballastMoving = false;

    bool leaks[4] =
    {
        false,
        false,
        false,
        false
    };

    const char *status = "READY";
};

//
//==============================================================
// Globals
//==============================================================
//

ControlState control;

TelemetryState telemetry;

int telemLeakCount = 0;

Adafruit_PWMServoDriver pca9685(PCA9685_I2C_ADDR);

bool pcaReady = false;

bool imuReady = false;

unsigned long lastSerialMs = 0;

unsigned long lastTelemetryMs = 0;

unsigned long lastStatusMs = 0;

#define RX_BUF 96

char rxBuf[RX_BUF];

int rxPos = 0;