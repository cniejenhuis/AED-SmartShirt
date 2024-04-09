#include <Wire.h>
#include <Math.h>
#include <Arduino_LSM9DS1.h>


#define MPU1_ADDRESS 0x68
#define MPU2_ADDRESS 0x69

// Complementary filter constants
#define alpha 0.98 // Weighting factor for accelerometer (0 < alpha < 1)

// Magnetometer calibration variables
int16_t magMinX = 32767, magMaxX = -32767;
int16_t magMinY = 32767, magMaxY = -32767;
int16_t magMinZ = 32767, magMaxZ = -32767;

// Magnetometer readings
int16_t magX1, magY1, magZ1;
int16_t magX2, magY2, magZ2;

// Yaw angles
float yaw1, yaw2;

void setup() 
{
        Wire.begin();
        Serial.begin(9600);
        
            //Initialize the Arduino nano 33 ble orientation
            if (!IMU.begin()) 
              {
              Serial.println("Failed to initialize IMU!");
              while (1);
              }
        // Initialize MPU9250 #1
        Wire.beginTransmission(MPU1_ADDRESS);
        Wire.write(0x6B); // PWR_MGMT_1 register
        Wire.write(0);    // Wake up MPU9250 #1
        Wire.endTransmission(true);
        
        // Initialize MPU9250 #2
        Wire.beginTransmission(MPU2_ADDRESS);
        Wire.write(0x6B); // PWR_MGMT_1 register
        Wire.write(0);    // Wake up MPU9250 #2
        Wire.endTransmission(true);

        // Perform magnetometer calibration
        calibrateMagnetometer();
}

void loop() 
{
       //Read orientation data from Arduino Nano 33 BLE
       if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) 
        {
              float accXnano, accYnano, accZnano;
              float gyroX, gyroY, gyroZ;
              float magX, magY, magZ;

              // Read accelerometer data Arduino Nano 33 BLE
              IMU.readAcceleration(accXnano, accYnano, accZnano);
              
              // Read gyroscope data Arduino Nano 33 BLE
              IMU.readGyroscope(gyroX, gyroY, gyroZ);
              
              // Read magnetometer data Arduino Nano 33 BLE
              IMU.readMagneticField(magX, magY, magZ);

              // Calculate pitch, roll, and yaw Arduino Nano 33 BLE
              float pitchNano = atan2(-accXnano, sqrt(accYnano * accYnano + accZnano * accZnano)) * 180.0 / PI;
              float rollNano = atan2(accYnano, accZnano) * 180.0 / PI;
              float magXcomp = magX * cos(rollNano) + magZ * sin(rollNano);
              float magYcomp = magX * sin(pitchNano) * sin(rollNano) + magY * cos(pitchNano) - magZ * sin(pitchNano) * cos(rollNano);
              float yawNano = atan2(-magYcomp, magXcomp) * 180.0 / PI;

              // Ensure yaw is within 0 to 360 degrees
              if (yawNano < 0) {
                yawNano += 360;
              }
              
              // Print the orientation angles Arduino Nano 33 BLE
              //Serial.print("pitchNano: ");
              Serial.print(pitchNano);
             // Serial.print(" rollNano: ");
              Serial.print(rollNano);
              //Serial.print(" yawNano: ");
              Serial.println(yawNano);

              //delay(100); // Adjust delay as needed
        }
             float pitchNano;
             float rollNano;
             float magXcomp;
             float magYcomp;
             float yawNano;
        // Read accelerometer data from MPU9250 #1
        int16_t accX1, accY1, accZ1;
        readAccelerometer(MPU1_ADDRESS, accX1, accY1, accZ1);

        // Read gyroscope data from MPU9250 #1
        int16_t gyroX1, gyroY1, gyroZ1;
        readGyroscope(MPU1_ADDRESS, gyroX1, gyroY1, gyroZ1);

        // Read magnetometer data from MPU9250 #1
        readMagnetometer(MPU1_ADDRESS, magX1, magY1, magZ1);

        // Read accelerometer data from MPU9250 #2
        int16_t accX2, accY2, accZ2;
        readAccelerometer(MPU2_ADDRESS, accX2, accY2, accZ2);

        // Read gyroscope data from MPU9250 #2
        int16_t gyroX2, gyroY2, gyroZ2;
        readGyroscope(MPU2_ADDRESS, gyroX2, gyroY2, gyroZ2);

        // Read magnetometer data from MPU9250 #2
        readMagnetometer(MPU2_ADDRESS, magX2, magY2, magZ2);
        
        // Calculate roll angle for MPU9250 #1
        float roll1 = atan2(accY1, accZ1) * 180.0 / M_PI;

        // Calculate pitch angle for MPU9250 #1
        float pitch1 = atan2(-accX1, sqrt(accY1 * accY1 + accZ1 * accZ1)) * 180.0 / M_PI;

        // Calculate yaw angle for MPU9250 #1
        float yaw1 = atan2(magY1 - (magMaxY + magMinY) / 2, magX1 - (magMaxX + magMinX) / 2) * 180.0 / M_PI;
        if (yaw1 < 0) {
          yaw1 += 360;
        }

        // Calculate roll angle for MPU9250 #2
        float roll2 = atan2(accY2, accZ2) * 180.0 / M_PI;

        // Calculate pitch angle for MPU9250 #2
        float pitch2 = atan2(-accX2, sqrt(accY2 * accY2 + accZ2 * accZ2)) * 180.0 / M_PI;

        // Calculate yaw angle for MPU9250 #2
        float yaw2 = atan2(magY2 - (magMaxY + magMinY) / 2, magX2 - (magMaxX + magMinX) / 2) * 180.0 / M_PI;
        if (yaw2 < 0) {
          yaw2 += 360;
        }
        
              // Calculate angle between MPU9250 #1 and Arduino Nano 33 BLE
              float rollAngleBetweenN1 = rollNano - roll1;
              float pitchAngleBetweenN1 = pitchNano - pitch1;
              float yawAngleBetweenN1 = yawNano - yaw1;

              // Ensure the angles are within -180 to 180 degrees range
              if (rollAngleBetweenN1 > 180) {
                rollAngleBetweenN1 -= 360;
              } else if (rollAngleBetweenN1 < -180) {
                rollAngleBetweenN1 += 360;
              }

              if (pitchAngleBetweenN1 > 180) {
                pitchAngleBetweenN1 -= 360;
              } else if (pitchAngleBetweenN1 < -180) {
                pitchAngleBetweenN1 += 360;
              }

              if (yawAngleBetweenN1 > 180) {
                yawAngleBetweenN1 -= 360;
              } else if (yawAngleBetweenN1 < -180) {
                yawAngleBetweenN1 += 360;
              }

              // Print angle between MPU9250 #1 and Arduino Nano 33 BLE
              Serial.print("Roll Angle MPU1 to Nano: ");
              Serial.println(rollAngleBetweenN1);
              Serial.print("Pitch Angle MPU1 to Nano: ");
              Serial.println(pitchAngleBetweenN1);
              Serial.print("Yaw Angle MPU1 to Nano: ");
              Serial.println(yawAngleBetweenN1);
                      Serial.println();


                    // Calculate angle between MPU9250 #2 and Arduino Nano 33 BLE
                    float rollAngleBetweenN2 = rollNano - roll2;
                    float pitchAngleBetweenN2 = pitchNano - pitch2;
                    float yawAngleBetweenN2 = yawNano - yaw2;

                    // Ensure the angles are within -180 to 180 degrees range
                    if (rollAngleBetweenN2 > 180) {
                      rollAngleBetweenN2 -= 360;
                    } else if (rollAngleBetweenN2 < -180) {
                      rollAngleBetweenN2 += 360;
                    }

                    if (pitchAngleBetweenN2 > 180) {
                      pitchAngleBetweenN2 -= 360;
                    } else if (pitchAngleBetweenN2 < -180) {
                      pitchAngleBetweenN2 += 360;
                    }

                    if (yawAngleBetweenN2 > 180) {
                      yawAngleBetweenN2 -= 360;
                    } else if (yawAngleBetweenN2 < -180) {
                      yawAngleBetweenN2 += 360;
                    }
                    
                    // Print angle between MPU9250 #2 and Arduino Nano 33 BLE
                    Serial.print("Roll Angle MPU2 to Nano: ");
                    Serial.println(rollAngleBetweenN2);
                    Serial.print("Pitch Angle MPU2 to Nano: ");
                    Serial.println(pitchAngleBetweenN2);
                    Serial.print("Yaw Angle MPU2 to Nano: ");
                    Serial.println(yawAngleBetweenN2);

        Serial.println();
        Serial.println();
        Serial.println();
        

        delay(500); // Adjust delay as needed
}

void readAccelerometer(uint8_t address, int16_t& accX, int16_t& accY, int16_t& accZ) {
  Wire.beginTransmission(address);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6, true);
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
}

void readGyroscope(uint8_t address, int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ) {
  Wire.beginTransmission(address);
  Wire.write(0x43); // Start with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6, true);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void readMagnetometer(uint8_t address, int16_t& magX, int16_t& magY, int16_t& magZ) {
  Wire.beginTransmission(address);
  Wire.write(0x03); // Start with register 0x03 (Mag X-axis MSB)
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6, true);
  magX = Wire.read() << 8 | Wire.read();
  magY = Wire.read() << 8 | Wire.read();
  magZ = Wire.read() << 8 | Wire.read();

  // Update min/max magnetometer values for calibration
  if (magX < magMinX) magMinX = magX;
  if (magX > magMaxX) magMaxX = magX;
  if (magY < magMinY) magMinY = magY;
  if (magY > magMaxY) magMaxY = magY;
  if (magZ < magMinZ) magMinZ = magZ;
  if (magZ > magMaxZ) magMaxZ = magZ;
}

void calibrateMagnetometer() {
  Serial.println("Starting magnetometer calibration...");
  Serial.println("Rotate the sensors in all directions for a few seconds.");

  unsigned long calibrationStartTime = millis();
  while (millis() - calibrationStartTime < 10000) { // Collect data for 10 seconds
    readMagnetometer(MPU1_ADDRESS, magX1, magY1, magZ1);
    readMagnetometer(MPU2_ADDRESS, magX2, magY2, magZ2);
    // You may also want to add delay here if necessary
  }

  Serial.println("Magnetometer calibration completed.");

  // Print calibration values
  Serial.print("Min X: "); Serial.println(magMinX);
  Serial.print("Max X: "); Serial.println(magMaxX);
  Serial.print("Min Y: "); Serial.println(magMinY);
  Serial.print("Max Y: "); Serial.println(magMaxY);
  Serial.print("Min Z: "); Serial.println(magMinZ);
  Serial.print("Max Z: "); Serial.println(magMaxZ);

}
