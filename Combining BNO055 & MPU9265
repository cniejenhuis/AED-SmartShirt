#include <Wire.h>
#include <Math.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// initialize a Madgwick filter:
Madgwick filter;
// sensor's sample rate is fixed at 104 Hz:
const float sensorRate = 20.00;

#define MPU1_ADDRESS 0x68
#define MPU2_ADDRESS 0x69
#define TCAADDR 0x70


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
  float X_MPU_Left, X_MPU_Right;


/* Assign a unique ID to this sensor at the same time */
Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55);
Adafruit_BNO055 bno3 = Adafruit_BNO055(55);

void displaySensorDetails(Adafruit_BNO055 *bno)
{
  sensor_t sensor;
  bno->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


void setup() 
{
        Wire.begin();
        Serial.begin(115200);


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

                      // attempt to start the Arduino Nano 33 BLE:
              if (!IMU.begin()) {
                Serial.println("Failed to initialize IMU");
                // stop here if you can't access the IMU:
                while (true);
              }
              // start the filter to run at the sample rate:
              filter.begin(sensorRate);

  Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the 1st BNO sensor */
  tcaselect(0);
  if (!bno1.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the 2nd BNO sensor */
  tcaselect(1);
  if (!bno2.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the 3rd BNO sensor */
  tcaselect(2);
  if (!bno3.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Display some basic information on each sensor */
  tcaselect(0);
  displaySensorDetails(&bno1); //Left
  tcaselect(1);
  displaySensorDetails(&bno2); //middle
  tcaselect(2);
  displaySensorDetails(&bno3); //Right
}

void loop() 
{
/////////////// ARDUINO NANO 33 BLE ///////////////////////////////////////////////////////////////////////////////////////////
  // values for acceleration & rotation:
  float accXnano, accYnano, accZnano;
  float gyroXnano, gyroYnano, gyroZnano;
  float magXnano, magYnano, magZnano;

  float acceleration[3], dps[3], magneticField[3];
    
  // values for orientation:
  float Z_Nano, Y_Nano, X_Nano;
  // check if the IMU is ready to read: 
  if (IMU.accelerationAvailable() &&
      IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    // read accelerometer & gyrometer:
    IMU.readAcceleration(accXnano, accYnano, accZnano);
    IMU.readGyroscope(gyroXnano, gyroYnano, gyroZnano);

      IMU.readAcceleration(accXnano, accYnano, accZnano);
      acceleration[0] = -accYnano;
      acceleration[1] = -accXnano;
      acceleration[2] = accZnano;
      IMU.readGyroscope(gyroXnano, gyroYnano, gyroZnano);
      dps[0] = -gyroYnano; 
      dps[1] = -gyroXnano; 
      dps[2] = gyroZnano;
      IMU.readMagneticField(magXnano, magYnano, magZnano);
      magneticField[0] = magXnano;  //magnetic North
      magneticField[1] = magYnano;  //West
      magneticField[2] = magZnano;  //Up

    accXnano = acceleration[0];
    accYnano = acceleration[1];
    accZnano = acceleration[2];

    gyroXnano = dps[0];
    gyroYnano = dps[1];
    gyroZnano = dps[2];

    magXnano = magneticField[0];
    magYnano = magneticField[1];
    magZnano = magneticField[2];
    
    // update the filter, which computes orientation:
    filter.update(gyroXnano, gyroYnano, gyroZnano, accXnano, accYnano, accZnano, magXnano, magYnano, magZnano);

    // print the X_Nano, Y_Nano and Z_Nano
    Z_Nano = filter.getRoll();
    Y_Nano = filter.getPitch();
    X_Nano = filter.getYaw();

    }
////// MPU 9265 SENSORS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// MPU 9265 SENSORS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// MPU 9265 Left /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// READ MPU LEFT
      // Read accelerometer data from MPU9250 #1
      int16_t accX1, accY1, accZ1;
      readAccelerometer(MPU1_ADDRESS, accX1, accY1, accZ1);
      // Read gyroscope data from MPU9250 #1
      int16_t gyroX1, gyroY1, gyroZ1;
      readGyroscope(MPU1_ADDRESS, gyroX1, gyroY1, gyroZ1);
      // Read magnetometer data from MPU9250 #1
      readMagnetometer(MPU1_ADDRESS, magX1, magY1, magZ1);

  //// CALCULATE MPU LEFT
      // Calculate roll angle for MPU9250 #1
      float Z_MPU_Left = atan2(accY1, accZ1) * 180.0 / M_PI;
      // Calculate pitch angle for MPU9250 #1
      float Y_MPU_Left = atan2(-accX1, sqrt(accY1 * accY1 + accZ1 * accZ1)) * 180.0 / M_PI;
      // Calculate yaw angle for MPU9250 #1
      float X_MPU_Left = atan2(magY1 - (magMaxY + magMinY) / 2, magX1 - (magMaxX + magMinX) / 2) * 180.0 / M_PI;
      if (X_MPU_Left < 0) {
        X_MPU_Left += 360;}


/////////////// MPU 9265 RIGHT /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //// READ MPU RIGHT
      // Read accelerometer data from MPU9250 #2
      int16_t accX2, accY2, accZ2;
      readAccelerometer(MPU2_ADDRESS, accX2, accY2, accZ2);
      // Read gyroscope data from MPU9250 #2
      int16_t gyroX2, gyroY2, gyroZ2;
      readGyroscope(MPU2_ADDRESS, gyroX2, gyroY2, gyroZ2);
      // Read magnetometer data from MPU9250 #2
      readMagnetometer(MPU2_ADDRESS, magX2, magY2, magZ2);

  //// CALCULATE MPU RIGHT
      // Calculate roll angle for MPU9250 #2
      float Z_MPU_Right = atan2(accY2, accZ2) * 180.0 / M_PI;
      // Calculate pitch angle for MPU9250 #2
      float Y_MPU_Right = atan2(-accX2, sqrt(accY2 * accY2 + accZ2 * accZ2)) * 180.0 / M_PI;
      // Calculate yaw angle for MPU9250 #2
      float X_MPU_Right = atan2(magY2 - (magMaxY + magMinY) / 2, magX2 - (magMaxX + magMinX) / 2) * 180.0 / M_PI;
      if (X_MPU_Right < 0) {
        X_MPU_Right += 360;
      }


////// BNO055 SENSORS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// BNO055 SENSORS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  sensors_event_t event1, event2, event3;

  // Left sensor
  tcaselect(0);
  bno1.getEvent(&event1);

  // Middle sensor
  tcaselect(1);
  bno1.getEvent(&event2);

  // Right sensor
  tcaselect(2);
  bno3.getEvent(&event3);

  // Calculate differences
  float X_diff_middle = event2.orientation.x - X_Nano;
  float Y_diff_middle = event2.orientation.y - Y_Nano;
  float Z_diff_middle = event2.orientation.z - Z_Nano;

  float X_diff_left = event1.orientation.x - X_diff_middle;
  float Y_diff_left = event1.orientation.y - Y_diff_middle;
  float Z_diff_left = event1.orientation.z - Y_diff_middle;

  float X_diff_right = X_diff_middle - event3.orientation.x;
  float Y_diff_right = Y_diff_middle - event3.orientation.y;
  float Z_diff_right = Z_diff_middle - event3.orientation.z;


    //// CALCULATE ANGLE MPU LEFT TO BNO LEFT  
        float Z_diff_MPU_left = Z_diff_left - Z_MPU_Left;
        float Y_diff_MPU_left = Y_diff_left - Y_MPU_Left;
        float X_diff_MPU_left = X_diff_left - X_MPU_Left;
        // Ensure the angles are within -180 to 180 degrees range
        if (Z_diff_MPU_left > 180) {
          Z_diff_MPU_left -= 360;} 
          else if (Z_diff_MPU_left < -180) {
          Z_diff_MPU_left += 360;}

        if (Y_diff_MPU_left > 180) {
          Y_diff_MPU_left -= 360;}
          else if (Y_diff_MPU_left < -180) {
          Y_diff_MPU_left += 360;}

        if (X_diff_MPU_left > 180) {
          X_diff_MPU_left -= 360;}
          else if (X_diff_MPU_left < -180) {
          X_diff_MPU_left += 360;}

  //// CALCULATE ANGLE MPU RIGHT TO Arduino Nano 33 BLE
          float Z_diff_MPU_right = Z_diff_right - Z_MPU_Right;
          float Y_diff_MPU_right = Y_diff_right - Y_MPU_Right;
          float X_diff_MPU_right = X_diff_right - X_MPU_Right;

          // Ensure the angles are within -180 to 180 degrees range
          if (Z_diff_MPU_right > 180) {
            Z_diff_MPU_right -= 360;}
            else if (Z_diff_MPU_right < -180) {
            Z_diff_MPU_right += 360;}

          if (Y_diff_MPU_right > 180) {
            Y_diff_MPU_right -= 360;}
            else if (Y_diff_MPU_right < -180) {
            Y_diff_MPU_right += 360;}

          if (X_diff_MPU_right > 180) {
            X_diff_MPU_right -= 360;}
            else if (X_diff_MPU_right < -180) {
            X_diff_MPU_right += 360;}

         
  Serial.println("LOWER BACK (arduino)");
    Serial.print("X: "); Serial.println(X_Nano);
    Serial.print("Y: "); Serial.println(Y_Nano);
    Serial.print("Z: "); Serial.println(Z_Nano);   
    Serial.println();

  Serial.println("BACK (bno055)");
    Serial.print("X: "); Serial.println(X_diff_middle);  // yaw
    Serial.print("Y: ");Serial.println(Y_diff_middle);  // pitch
    Serial.print("Z: "); Serial.println(Z_diff_middle);  // roll
    Serial.println();

  Serial.println("LEFT UPPER ARM (bno055)");
    Serial.print("X: "); Serial.println(X_diff_left); // yaw
    Serial.print("Y: "); Serial.println(Y_diff_left); // pitch
    Serial.print("Z: "); Serial.println(Z_diff_left); // roll
    Serial.println();

  Serial.println("LEFT LOWER ARM (mpu9265)");
    Serial.print("X: "); Serial.println(X_diff_MPU_left);
    Serial.print("Y: "); Serial.println(Y_diff_MPU_left);
    Serial.print("Z: "); Serial.println(Z_diff_MPU_left);
    Serial.println();

  Serial.println("RIGHT UPPER ARM (bno055)");
    Serial.print("X: "); Serial.println(X_diff_right); // yaw
    Serial.print("Y: "); Serial.println(Y_diff_right); // pitch
    Serial.print("Z: "); Serial.println(Z_diff_right); // roll
    Serial.println();

  Serial.println("RIGHT LOWER ARM (mpu9265)");
      Serial.print("X: "); Serial.println(X_diff_MPU_right);   
    Serial.print("Y: "); Serial.println(Y_diff_MPU_right);        
    Serial.print("Z: "); Serial.println(Z_diff_MPU_right);



    
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
