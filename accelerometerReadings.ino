#include <Wire.h>
#include <math.h>

// I2C address for the GY-511 (LSM303DLHC)
#define LSM303_ADDRESS_ACCEL 0x19

// Relays 
const int relayInput1 = 6; 
const int relayInput2 = 7; 
float trunkAngleTarget = 0; 

// calibration
float zeroAngle = 0; // variable to store the calibrated zero angle  

//medium sized values as everything will be moving slow 
float Kp = 2.0;   // Proportional gain 
float Ki = 0.5;   // Integral gain
float Kd = 0.2;   // Derivative gain


//----------------manually change per user---------------------------------------
const float userWeight = 68.0 ; // kg 
const float userTrunk = 0.6 ; // m 
const float userHead = 0.2 ; // m 

float prevError = 0; 
float integral = 0; 


// ---------------- actuator functions ----------------
void actuatorForward() {
  digitalWrite(relayInput1, LOW);
  digitalWrite(relayInput2, HIGH);
}

void actuatorStop(){
  digitalWrite(relayInput1, LOW); 
  digitalWrite(relayInput2, LOW);
}

void actuatorBackward(){
  digitalWrite(relayInput1, HIGH); 
  digitalWrite(relayInput2, LOW);
}

// ------------------------- read raw accelerometer -------------------------------
void readAccel(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(LSM303_ADDRESS_ACCEL);
  Wire.write(0x28 | 0x80); // OUT_X_L_A with auto-increment
  Wire.endTransmission(false);
  Wire.requestFrom(LSM303_ADDRESS_ACCEL, 6, true);

  int16_t xl = Wire.read();
  int16_t xh = Wire.read();
  int16_t yl = Wire.read();
  int16_t yh = Wire.read();
  int16_t zl = Wire.read();
  int16_t zh = Wire.read();

  x = (xh << 8) | xl;
  y = (yh << 8) | yl;
  z = (zh << 8) | zl;
}

//-----------------------------accelerometer calibration---------------------------------
float callibration() { 
  delay(200); 

  float sumAngle = 0.0 ; 
  const int samples = 100 ; 

  for (int i = 0; i < samples; i++) {
    int16_t axRaw, ayRaw, azRaw;
    readAccel(axRaw, ayRaw, azRaw);

    // convert to g (±2g, 16-bit)
    float ay = ayRaw / 16384.0;
    float az = azRaw / 16384.0;

    float angle = atan2(ay, az) * 180.0 / 3.14159;
    sumAngle += angle;

    delay(10);
  }

  zeroAngle = sumAngle / samples; 

  actuatorForward(); 
  delay(100); // small extension
  actuatorStop(); 

  return zeroAngle; 
}

// --------------------------------angle relative to the zero---------------------------------
float readCurrentAngle() {
  int16_t axRaw, ayRaw, azRaw;
  readAccel(axRaw, ayRaw, azRaw);

  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float angle = atan2(ay, az) * 180.0 / PI;

  return angle - zeroAngle; 
} 

// ---------------- setup ----------------
void setup() {
  Serial.begin(9600);
  Wire.setClock(20000);
  pinMode(relayInput1, OUTPUT);
  pinMode(relayInput2, OUTPUT);

  Wire.begin(); // A4 = SDA, A5 = SCL automatically

  // initialize accelerometer: CTRL_REG1_A = 0x27 (enable X/Y/Z, 10Hz)
  Wire.beginTransmission(LSM303_ADDRESS_ACCEL);
  Wire.write(0x20); // CTRL_REG1_A
  Wire.write(0x27); // normal mode, all axes enabled
  Wire.endTransmission();

  Serial.println("Starting automatic calibration...");
  float zero = callibration();
  Serial.print("Calibration done. Zero angle = ");
  Serial.println(zero);
}

// ---------------- loop ----------------
void loop() {
  float currentAngle = abs(readCurrentAngle());
  Serial.print(currentAngle);
  Serial.println(" degrees");

  delay(200);



}