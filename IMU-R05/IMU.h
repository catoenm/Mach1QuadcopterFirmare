// -------------------- SW110-IMULibrary-R05 -------------------- //

#ifndef IMU_h
#define IMU_h

#include <Wire.h>
#include <Arduino.h>

// Addresses
#define GYRO 0b1101011 // Gyro (L3GD20H) address address w/o R/W bit
#define ACCEL 0b0011101 // Accelerometer/Magnetometer (LSM303D) address address w/o R/W bit
#define ALT 0b1011101 // Altimeter (LPS331AP) address w/o R/W bit
#define WHO_AM_I 0x0F // Same WHO_AM_I register for all devices
#define RAW_TO_G 0.00006103515625
#define RAW_TO_GAUSS 0.0001220703125
#define RAW_TO_RAD_PER_SEC 0.0001304948934
#define RAW_TO_DEG_PER_SEC 0.00747680663
#define RAD_TO_DEG 57.295779513
#define X 0
#define Y 1
#define Z 2

class IMU {
    private:
        bool accelerometerON;
        bool gyroscopeON;
        bool altimeterON;
        int16_t accel[3];
        int16_t newAccel[3];
        int16_t mag[3];
        int16_t newMag[3];
        int16_t gyro[3];
        int16_t newGyro[3];
    
        float gyroIntegrator[3]; // angle in degrees from gyro perspective
        float accelLowPassCoeff; // proportion of new value (1 = no filter)
        float magLowPassCoeff;
        float gyroLowPassCoeff;
        float gyroPercentRoll; // percentage of gyro used
        float gyroPercentPitch;
        float gyroPercentYaw;
    
        long int prevTime;
    
    public:
        // Setup Methods
        bool enableAccelerometer();
        bool enableGyroscope();
        bool enableAltimeter();
        bool enable();
        void calibrateGyro();
        void setAccelLowPassCoeff(float value);
        void setMagLowPassCoeff(float value);
        void setGyroLowPassCoeff(float value);
        void setGyroPercentRoll(float value);
        void setGyroPercentPitch(float value);
        void setGyroPercentYaw(float value);
        /*
        bool disableAccelerometer() const;
        bool disableGyroscope() const;
        bool disableAltimeter() const;
        */
        // Accessors
        void readIMU();
        float pressure();
        float temperature();
        float getRoll();
        float getPitch();
        float getYaw();
        float getAccel(int axis);
        float getMag(int axis);
        float getGyro(int axis);
        // Constructor
        IMU();
    
    private:
        // I2C Methods
        void writeToReg(byte deviceAddress, byte regAddress, byte value) const;
        bool testComms(byte address, byte whoAmIAnswer) const;
        void readAcceleration();
        void readMagnetometer();
        void readGyroscope();
        int getRawPressure();
        int getRawTemperature();
};

#endif

