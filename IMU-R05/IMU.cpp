// -------------------- SW110-IMULibrary-R05 -------------------- //

#include "imu.h"

IMU::IMU() {
    Wire.begin();
    accelerometerON = false;
    gyroscopeON = false;
    altimeterON = false;
    accelLowPassCoeff = 0.5;
    magLowPassCoeff = 0.5;
    gyroLowPassCoeff = 0.5;
    gyroPercentRoll = 0.5;
    gyroPercentPitch = 0.5;
    gyroPercentYaw = 1.0;
    
    for (int i = 0; i < 3; i++) {
        gyroIntegrator[i] = 0;
    }
    prevTime = 0;
}

void IMU::writeToReg(byte deviceAddress, byte regAddress, byte value) const {
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(value);
    Wire.endTransmission();
}

bool IMU::testComms(byte address, byte whoAmIAnswer) const {
    Wire.beginTransmission(address);
    Wire.write(WHO_AM_I);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    if (Wire.available() && Wire.read() == whoAmIAnswer)
        return true;
    return false;
}

bool IMU::enableAccelerometer() {
    bool comm = testComms(ACCEL,0x49);
    writeToReg(ACCEL,0x20,0x87); // CTRL_1 -> 1600 Hz accelerometer output, All axes enbabled
    writeToReg(ACCEL,0x24,0x14); // CTRL_5 -> 100 Hz magnetometer output
    writeToReg(ACCEL,0x25,0x00); // CTRL_6 -> 2 Gauss range
    writeToReg(ACCEL,0x26,0x00); // CTRL_7 -> Magnetometer mode = continous-conversion
    if(comm) {
        accelerometerON = true;
        return true;
    }
    accelerometerON = false;
    return false;
}

bool IMU::enableGyroscope() {
    bool comm = testComms(GYRO,0xD7);
    writeToReg(GYRO,0x20,0x0F); // CTRL_1 -> TBC
    if(comm) {
        gyroscopeON = true;
        return true;
    }
    gyroscopeON = false;
    return false;
}

bool IMU::enableAltimeter() {
    bool comm = testComms(ALT,0xBB);
    writeToReg(ALT,0x20,0xE0); // CTRL_1 -> TBC
    if(comm) {
        altimeterON = true;
        return true;
    }
    altimeterON = false;
    return false;
}

bool IMU::enable() {
    return enableAccelerometer() && enableGyroscope() && enableAltimeter();
}

void IMU::setAccelLowPassCoeff(float value) {
    accelLowPassCoeff = value;
}

void IMU::setMagLowPassCoeff(float value) {
    magLowPassCoeff = value;
}

void IMU::setGyroLowPassCoeff(float value) {
    gyroLowPassCoeff = value;
}

void IMU::setGyroPercentRoll(float value) {
    gyroPercentRoll = value;
}

void IMU::setGyroPercentPitch(float value) {
    gyroPercentPitch = value;
}

void IMU::setGyroPercentYaw(float value) {
    gyroPercentYaw = value;
}

/*
 bool IMU::disableAccelerometer() const {
 
 }
 
 bool IMU::disableGyroscope() const {
 
 }
 
 bool IMU::disableAltimeter() const {
 
 }
 */

void IMU::readAcceleration() {
    // Setup
    Wire.beginTransmission(ACCEL);
    Wire.write(0x28 | (1 << 7));  // starting with register 0x28 (OUT_X_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(ACCEL,(byte)6);  // request a total of 6 registers
    while (Wire.available() < 6);
    for (int i = 0; i < 3; i++){
        newAccel[i] = (int16_t)(Wire.read() | Wire.read()<<8); // read x, y & z axes
    }
}

void IMU::readMagnetometer() {
    // Setup
    Wire.beginTransmission(ACCEL);
    Wire.write(0x08 | (1 << 7));  // starting with register 0x08 (OUT_X_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(ACCEL,(byte)6);  // request a total of 6 registers
    while (Wire.available() < 6);
    for (int i = 0; i < 3; i++){
        newMag[i] = (int16_t)(Wire.read() | Wire.read()<<8); // read x, y & z axes
    }
}

void IMU::readGyroscope() {
    // Setup
    Wire.beginTransmission(GYRO);
    Wire.write(0x28 | (1 << 7));  // starting with register 0x28 (OUT_X_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(GYRO,(byte)6);  // request a total of 6 registers
    while (Wire.available() < 6);
    for (int i = 0; i < 3; i++){
        newGyro[i] = (int16_t)(Wire.read() | Wire.read()<<8) - 70; // read x, y & z axes
    }
}

int IMU::getRawPressure() {
    // Setup
    Wire.beginTransmission(ALT);
    Wire.write(0x28 | (1 << 7));  // starting with register 0x28 (PRESS_OUT_XL), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(ALT,(byte)3);  // request a total of 5 registers
    while (Wire.available() < 3);
    uint8_t pxl = Wire.read();
    uint8_t pl = Wire.read();
    uint8_t ph = Wire.read();
    return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

int IMU::getRawTemperature() {
    // Setup
    Wire.beginTransmission(ALT);
    Wire.write(0x2B | (1 << 7));  // starting with register 0x2B (TEMP_OUT_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(ALT,(byte)2);  // request a total of 5 registers
    while (Wire.available() < 2);
    uint8_t tl = Wire.read();
    uint8_t th = Wire.read();
    return (int16_t)(th << 8 | tl);
}

float IMU::getAccel(int axis) {
    return accel[axis] * RAW_TO_G;
}

float IMU::getMag(int axis) {
    return mag[axis] * RAW_TO_GAUSS;
}

float IMU::getGyro(int axis) {
    return gyro[axis] * RAW_TO_DEG_PER_SEC;
}

void IMU::calibrateGyro() {
    float temp = gyroPercentYaw;
    gyroPercentYaw = 0.0;
    gyroIntegrator[Z] = getYaw();
    gyroPercentYaw = temp;
}

void IMU::readIMU() {
    // Read Values
    readAcceleration();
    readMagnetometer();
    readGyroscope();
    
    // Update time;
    long int currTime = millis();
    
    // Low pass filter & gyro integrator
    for (int i = 0; i < 3; i++) {
        // Low pass filter: updatedValue = old*(1-coeff) + new*coeff
        accel[i] = accel[i] * (1.0 - accelLowPassCoeff) + newAccel[i] * accelLowPassCoeff;
        mag[i] = mag[i] * (1.0 - magLowPassCoeff) + newMag[i] * magLowPassCoeff;
        gyro[i] = gyro[i] * (1.0 - gyroLowPassCoeff) + newGyro[i] * gyroLowPassCoeff;
        // Update gyro integrator
        gyroIntegrator[i] += gyro[i] * RAW_TO_RAD_PER_SEC * (currTime - prevTime) / 1000.0;
    }
    
    // Set gyro range to -180 to 180 deg (same as compass output)
    if (gyroIntegrator[2] > M_PI)
        gyroIntegrator[2] = -M_PI+0.00001; // add tolerance to avoid boucing back an forth
    else if (gyroIntegrator[2] < -M_PI)
        gyroIntegrator[2] = M_PI-0.00001;
    
    // Update time
    prevTime = currTime;
}

float IMU::pressure() {
    return double(getRawPressure()) / 4096; // measured in mPa
}

float IMU::temperature() {
    return double(getRawTemperature()) / 480 + 42.5; // measured in degreeC
}


float IMU::getRoll() {
    return -1 * RAD_TO_DEG * atan2((-1.0)*getAccel(Y), getAccel(Z));
}

float IMU::getPitch() {
    return -1 * RAD_TO_DEG * atan2(getAccel(X), sqrt(getAccel(Y)*getAccel(Y)
                              + getAccel(Z)*getAccel(Z)));
}

float IMU::getYaw()  {
    float roll = getRoll() / RAD_TO_DEG;
    float pitch = getPitch() / RAD_TO_DEG;
    return (atan2(  (getMag(Z) * sin(roll))
                  - (getMag(Y) * cos(roll)),
                    (getMag(X) * cos(pitch))
                  + (getMag(Y) * sin(pitch) * sin(roll))
                  + (getMag(Z) * sin(pitch) * cos(roll)))
            * (1.0 - gyroPercentYaw) + gyroIntegrator[Z] * gyroPercentYaw) * RAD_TO_DEG;
}
