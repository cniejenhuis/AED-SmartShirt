const int sensorPin = A1;    // Analog pin connected to the variable resistor
const float Resistor = 10000.0;  // Resistance of the fixed resistor in ohms
const int calibrateMaxButtonPin = 7;  // Pin connected to the button for calibrating max value
const int calibrateMinButtonPin = 3;  // Pin connected to the button for calibrating min value
const int ledPin = 6; // Pin connected to the LED (changed from D2 to 2)

float maxVoltage = 0.0; // Max voltage at 160 degrees
float minVoltage = 5.0; // Min voltage at 0 degrees

bool maxButtonPressed = false; // Flag to indicate max button press
bool minButtonPressed = false; // Flag to indicate min button press

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(calibrateMaxButtonPin, INPUT_PULLUP); // Set pin mode for max calibration button
  pinMode(calibrateMinButtonPin, INPUT_PULLUP); // Set pin mode for min calibration button
  pinMode(ledPin, OUTPUT); // Set pin mode for LED
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Read the analog value from the sensor pin
  float voltage = sensorValue * (5.0 / 1023.0); // Convert analog value to voltage (5V Arduino)
  
  // Check for max calibration button press
  if (digitalRead(calibrateMaxButtonPin) == HIGH && !maxButtonPressed) {
    maxVoltage = voltage;
    Serial.println("Max angle calibrated.");
    maxButtonPressed = true;
    delay(1000); // Debouncing delay
  } else if (digitalRead(calibrateMaxButtonPin) == LOW) {
    maxButtonPressed = false;
  }
  
  // Check for min calibration button press
  if (digitalRead(calibrateMinButtonPin) == HIGH && !minButtonPressed) {
    minVoltage = voltage;
    Serial.println("Min angle calibrated.");
    minButtonPressed = true;
    delay(1000); // Debouncing delay
  } else if (digitalRead(calibrateMinButtonPin) == LOW) {
    minButtonPressed = false;
  }
  
  // Calculate the percentage based on calibrated min and max voltages
  float Angle = map(sensorValue, minVoltage / (5.0 / 1023.0), maxVoltage / (5.0 / 1023.0), 0, 140);

  // Send angle data to serial plotter
  Serial.println(Angle);

  // Print angle
  Serial.print("Angle: ");
  //Serial.print(Angle);
  Serial.println(" degrees ");

  // Control LED based on angle
  if (Angle > 160) {
    digitalWrite(ledPin, HIGH); // Turn on LED
  } else {
    digitalWrite(ledPin, LOW); // Turn off LED
  }

  delay(50); // Delay for stability
}