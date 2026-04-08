const int sensorPin = A2;     
const int threeVoltPin = A0;    // PWM pin for ~3V output

const int analogMax = 1023;   
const float VinMax = 3.0;     

const int numPoints = 6;      

float voltage[numPoints];
float force[numPoints];
float lnF[numPoints];
float lnV[numPoints];
float sumLnV2 = 0; // this will hold Σ(lnV^2)
float sumLnVF = 0; // this will hold Σ(lnF^2)
// Single variables for sums
float sumLnV = 0;
float sumLnF = 0;
float k = 0 ; 
float lnK = 0 ;
float n = 0 ; 



float readAverageVoltage(int samples = 50) {
  float total = 0;
  for(int i = 0; i < samples; i++){
    int raw = analogRead(sensorPin);
    float V = (raw / float(analogMax)) * VinMax;
    total += V;
    delay(10);
  }
  return total / samples;
}

float waitForForceInput() {
  while(!Serial.available()) {}
  float value = Serial.parseFloat();
  while(Serial.available()) Serial.read();
  return value;
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  pinMode(sensorPin, INPUT);
  pinMode(threeVoltPin, OUTPUT);

  // output constant ~3V
  analogWrite(threeVoltPin, 153); // 153/255*5V ≈ 3V

  Serial.println("=== FlexiForce 6-Point Data Capture ===");

  for(int i = 0; i < numPoints; i++) {
    Serial.print("Place weight #");
    Serial.print(i+1);
    Serial.println(" and enter force in kg:");
    
    force[i] = waitForForceInput();
    while(force[i] <= 0) {  
      Serial.println("Force must be greater than 0. Try again:");
      force[i] = waitForForceInput();
    }
    
    delay(3000);  // stabilize
    voltage[i] = readAverageVoltage();
    
    Serial.print("Measured Voltage: ");
    Serial.println(voltage[i], 4);
    Serial.println("---------------------------");
  }
  
  Serial.println("=== All Voltage-Force Pairs ===");
  for(int i = 0; i < numPoints; i++){
    Serial.print("Force (kg): ");
    Serial.print(force[i], 2);
    Serial.print("  |  Voltage (V): ");
    Serial.println(voltage[i], 4);
  }


  // Compute ln(voltages) and sum
  sumLnV = 0; // reset sum
  for (int i = 0; i < numPoints; i++) {
    lnV[i] = log(voltage[i]);
    sumLnV += lnV[i];
  }

  // Compute ln(forces) and sum
  sumLnF = 0; // reset sum
  for (int i = 0; i < numPoints; i++) {
    lnF[i] = log(force[i]);
    sumLnF += lnF[i];
  }

  // Optional: print results
  Serial.println("ln(Voltages):");
  for (int i = 0; i < numPoints; i++) Serial.println(lnV[i], 6);
  Serial.print("Sum of ln(Voltages): ");
  Serial.println(sumLnV, 6);

  Serial.println("ln(Forces):");
  for (int i = 0; i < numPoints; i++) Serial.println(lnF[i], 6);
  Serial.print("Sum of ln(Forces): ");
  Serial.println(sumLnF, 6);


  

  for (int i = 0; i < numPoints; i++) {
    sumLnV2 += lnV[i] * lnV[i]; // multiply element by itself and add
  }
  for (int i = 0; i < numPoints; i++) {
    sumLnVF += lnV[i] * lnF[i]; // multiply element by itself and add
  }

  n = ((numPoints * sumLnVF) - (sumLnV * sumLnF)) /
    ((numPoints * sumLnV2) - (sumLnV * sumLnV)); 

  Serial.print("n = ");
  Serial.println(n, 6);

  Serial.print("Sum of lnV squared: ");
  Serial.println(sumLnV2, 6);
  Serial.print("Sum of lnVF : ");
  Serial.println(sumLnVF, 6);

  lnK = (sumLnF - (n*sumLnV) )/ numPoints;
  Serial.print("lnK = ");
  Serial.println(lnK, 6);

  k = exp(lnK);
  Serial.print("k = ");
  Serial.println(k, 6);


}

void loop() {
  // nothing here
  analogWrite(threeVoltPin, 153) ; //3 volts output 
  // ---------------- Sensor 1 ----------------
  float V1 = readAverageVoltage();
  float force1 = k * pow(V1, n);
  Serial.print(V1);
  Serial.print("   |   ");
  Serial.println(force1) ; 
}