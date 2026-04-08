#include <Wire.h>

// HMC5883L I2C address
#define HMC5883L_ADDRESS 0x1E

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(9600); // Start serial communication for debugging

  // Initialize the HMC5883L sensor
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00); // Select configuration register A
  Wire.write(0x70); // Set 8-average, 15 Hz default, normal measurement
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x01); // Select configuration register B
  Wire.write(0xA0); // Set gain = 5
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

void loop() {
  int16_t x, y, z;

  // Request 6 bytes of data from the sensor
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03); // Start reading from data output X MSB register
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDRESS, 6);

  if (Wire.available() == 6) {
    x = (Wire.read() << 8) | Wire.read(); // Combine MSB and LSB for X-axis
    z = (Wire.read() << 8) | Wire.read(); // Combine MSB and LSB for Z-axis
    y = (Wire.read() << 8) | Wire.read(); // Combine MSB and LSB for Y-axis
  }

  // Print the raw magnetometer data
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  delay(500); // Wait for 500ms before the next reading
}