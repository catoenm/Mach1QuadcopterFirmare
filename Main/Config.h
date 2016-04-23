#include <SPI.h>
#include <RH_RF69.h>
#include <ServoTimer2.h>
#include <PID_v1.h>

//Cyclone Robotics Mach 1 Flight Controller Configuration File

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

