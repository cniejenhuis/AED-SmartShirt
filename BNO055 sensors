#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define TCAADDR 0x70

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

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  Wire.begin();

  /* Initialise the 1st sensor */
  tcaselect(0);
  if (!bno1.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the 2nd sensor */
  tcaselect(1);
  if (!bno2.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the 3rd sensor */
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

void loop(void)
{
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
  float X_diff_left = event1.orientation.x - event2.orientation.x;
  float Y_diff_left = event1.orientation.y - event2.orientation.y;
  float Z_diff_left = event1.orientation.z - event2.orientation.z;

  float X_diff_right = event2.orientation.x - event3.orientation.x;
  float Y_diff_right = event2.orientation.y - event3.orientation.y;
  float Z_diff_right = event2.orientation.z - event3.orientation.z;

  Serial.println();
  Serial.println();
  Serial.println();

  // Print differences
  Serial.println("Left BNO055 Sensor");
  Serial.print("X: "); Serial.println(X_diff_left); // yaw
  Serial.print("Y: "); Serial.println(Y_diff_left); // pitch
  Serial.print("Z: "); Serial.println(Z_diff_left); // roll

  Serial.println();

  Serial.println("Middle BNO055 Sensor");
  Serial.print("X: "); Serial.println( event2.orientation.x);  // yaw
  Serial.print("Y: ");Serial.println(-event2.orientation.y);  // pitch
  Serial.print("Z: "); Serial.println(-event2.orientation.z);  // roll

  Serial.println();

  Serial.println("Right BNO055 Sensor");
  Serial.print("X: "); Serial.println(X_diff_right); // yaw
  Serial.print("Y: "); Serial.println(Y_diff_right); // pitch
  Serial.print("Z: "); Serial.println(Z_diff_right); // roll


  delay(500);
}

