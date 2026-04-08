const int flexiForcePin = A2;
const float VinMax = 3.3;      // Change to 5.0 if using 5V
const int analogMax = 1023;

float m = 0;   // slope
float b = 0;   // intercept

float readAverageVoltage(int samples = 50) {
  float total = 0;
  for (int i = 0; i < samples; i++) {
    int raw = analogRead(flexiForcePin);
    float V = (raw / float(analogMax)) * VinMax;
    total += V;
    delay(10);
  }
  return total / samples;
}

void waitForEnter() {
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  Serial.println("=== FlexiForce 2-Point Calibration ===");

  // ---- First Calibration Point ----
  Serial.println("Remove all weight. Press ENTER.");
  waitForEnter();
  float V1 = readAverageVoltage();
  float F1 = 0.0;   // No load

  Serial.print("Baseline Voltage: ");
  Serial.println(V1, 4);

  // ---- Second Calibration Point ----
  Serial.println("Place known weight (in Newtons).");
  Serial.println("Enter force value in N and press ENTER:");

  while (!Serial.available()) {}
  float F2 = Serial.parseFloat();
  while (Serial.available()) Serial.read();

  delay(3000);  // give time to stabilize

  float V2 = readAverageVoltage();

  Serial.print("Measured Voltage: ");
  Serial.println(V2, 4);

  // ---- Calculate Calibration ----
  m = (F2 - F1) / (V2 - V1);
  b = F1 - m * V1;

  Serial.println("Calibration Complete!");
  Serial.print("Slope (m): ");
  Serial.println(m, 6);
  Serial.print("Intercept (b): ");
  Serial.println(b, 6);

  Serial.println("Now reading force...");
}

void loop() {
  float V = readAverageVoltage(20);
  float force = m * V + b;

  Serial.print("Voltage: ");
  Serial.print(V, 3);
  Serial.print(" V  |  Force: ");
  Serial.print(force, 2);
  Serial.println(" N");

  delay(200);
}