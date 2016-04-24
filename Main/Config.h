#include <SPI.h>
#include <RH_RF69.h>
#include <ServoTimer2.h>
#include <PID_v1.h>
#include <Wire.h>
#include "IMU.h"

//Cyclone Robotics Mach 1 Flight Controller Configuration File

//PID
#define ROLL_P 0
#define ROLL_I 0
#define ROLL_D 0
#define ROLL_MAX 100
#define ROLL_MIN 0

#define PITCH_P 0
#define PITCH_I 0
#define PITCH_D 0
#define PITCH_MAX 100
#define PITCH_MIN 0

#define YAW_P 0
#define YAW_I 0
#define YAW_D 0
#define YAW_MAX 100
#define YAW_MIN 0

//ESC'S
#define ESC1 3
#define ESC2 5
#define ESC3 6
#define ESC4 9

//LED'S
#define RED 7
#define GREEN 8
#define PORT A1
#define STAR A2

//INTERRUPTS
#define INTER 2

//BUTTONS
#define BUTTON A0

//UNDEFINED
#define TB1 -1
#define TB2 -1

//BATTERY
#define VBAT A3
#define VBAT_CONVERT 0.01270772239

//SERIAL
#define TX 0
#define RX 1

//SERVO
#define SERVO 10

//I2C
#define I2C_SDA A4
#define I2C_CLK A5

//SPI
#define SPI_SS 4
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_CLK 13

//Other

#define VBAT_CONVERT 0.01270772239
#define FULL_THROTTLE 1860
#define ZERO_THROTTLE 1060
#define THROTTLE_RANGE 800

//Comm
RH_RF69 rf69 (4);
uint16_t userMessage;
uint16_t upperThrottle;
uint8_t lowerThrottle;

//Motor function
ServoTimer2 motors[4];

const int maxThrottle = 30;
const double spinTime = 15000;
const bool sweep = true;

double motor0Out = 0;
double motor1Out = 0;
double motor2Out = 0;
double motor3Out = 0;

double throttle = 0;

//Led
long ledTimeStamp = 0;
long spinTimeStamp = 0; 

//Pid
double rollOutput;
double rollInput;
double rollSetpoint;
PID    rollPid(&rollInput, &rollOutput, &rollSetpoint, ROLL_P, ROLL_I, ROLL_D, DIRECT);

double pitchOutput;
double pitchInput;
double pitchSetpoint;
PID    pitchPid(&pitchInput, &pitchOutput, &pitchSetpoint, PITCH_P, PITCH_I, PITCH_D, DIRECT);

double yawOutput;
double yawInput;
double yawSetpoint;
PID    yawPid(&yawInput, &yawOutput, &yawSetpoint, ROLL_P, ROLL_I, ROLL_D, DIRECT);

//Inputs
IMU imu;

double Imu_Roll;
double Imu_Pitch;
double Imu_Yaw;

//State Machines
enum MotorState {
  startSpinning,
  spinning,
  idle
};

enum ArmLEDState {
  on,
  off
};

ArmLEDState armLEDState;
MotorState motorState;




