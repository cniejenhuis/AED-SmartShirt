#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define TCAADDR 0x70

/* Assign a unique ID to this sensor at the same time */
Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56);  // Ensure unique IDs for each sensor
Adafruit_BNO055 bno3 = Adafruit_BNO055(57);  // Ensure unique IDs for each sensor

void displaySensorDetails(Adafruit_BNO055 *bno) {
  sensor_t sensor;
  bno->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" uT");
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

void setup(void) {
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  Wire.begin();

  /* Initialise the 1st sensor */
  tcaselect(2);
  if (!bno1.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the 2nd sensor */
  tcaselect(0);
  if (!bno2.begin()) {
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while (1);
  }

  /* Initialise the 3rd sensor */
  tcaselect(1);
  if (!bno3.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while (1);
  }

  /* Display some basic information on each sensor */
  tcaselect(2);
  displaySensorDetails(&bno1);
  tcaselect(0);
  displaySensorDetails(&bno2);
  tcaselect(1);
  displaySensorDetails(&bno3);
}

void loop(void) {
  imu::Quaternion humerus_l_imu, torso_imu, humerus_r_imu;

  // Left sensor
  tcaselect(2);
  humerus_l_imu = bno1.getQuat();

  // Middle sensor
  tcaselect(0);
  torso_imu = bno2.getQuat();

  // Right sensor
tcaselect(1);
humerus_r_imu = bno3.getQuat();

  // Send quaternion data over Serial in the format qx,qy,qz,qw for each sensor
  Serial.print(millis() / 1000.0);  // Time in seconds
  Serial.print("\t");
  Serial.print(humerus_l_imu.x());
  Serial.print(",");
  Serial.print(humerus_l_imu.y());
  Serial.print(",");
  Serial.print(humerus_l_imu.z());
  Serial.print(",");
  Serial.print(humerus_l_imu.w());
  Serial.print("\t");
  Serial.print(torso_imu.x());
  Serial.print(",");
  Serial.print(torso_imu.y());
  Serial.print(",");
  Serial.print(torso_imu.z());
  Serial.print(",");
  Serial.print(torso_imu.w());
  Serial.print("\t");
  Serial.print(humerus_r_imu.x());
  Serial.print(",");
  Serial.print(humerus_r_imu.y());
  Serial.print(",");
  Serial.print(humerus_r_imu.z());
  Serial.print(",");
  Serial.print(humerus_r_imu.w());
  Serial.println();

  delay(500);
}
