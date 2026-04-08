// force sensors 
const int forceSensor1 = A2;
const int forceSensor2 = A3 ; 
const int threevolts = A0; 
const float Vref = -3.0;             // Voltage applied to inverting input (Vref)
const float Rf = 10000.0;          // Feedback resistor in ohms
const int analogMax = 1023;         // 10-bit ADC
const float VinMax = 3.0;          // Max voltage at Arduino pin


// ----------smoothing values----------------
float smoothValue = 0;
float alpha = 0.2;   // smoothing factor (0.05–0.2 works well)


float k = 0; 
float lnK = 0;
float n = 0 ;



//-------------------calibrated values based on Power Law equation-------------------------
const int numPoints = 6;

//array storage for voltages and forces (f-sensequadcallibration.ino)
/*
Force (kg): 0.10  |  Voltage (V): 2.2227
Force (kg): 0.50  |  Voltage (V): 2.2048
Force (kg): 2.30  |  Voltage (V): 2.1731
Force (kg): 6.80  |  Voltage (V): 2.1233
Force (kg): 11.33  |  Voltage (V): 2.1175
Force (kg): 3.00  |  Voltage (V): 2.1642 */ 
const float voltages[numPoints] = {2.2227, 2.2048, 2.1731, 2.1233, 2.1175, 2.1642};
const float forces[numPoints]   = {0.1, 0.5, 2.3, 6.8, 11.33, 3};




//----------slight avg for force sensor---------------------
const int sampleCount = 10;         // Number of samples for moving average
float readings[sampleCount];
float readings1[sampleCount] ; 
int readIndex = 0;
int readIndex1  = 0 ; 
float total = 0;
float total1 = 0; 
float average = 0;


float lnF[numPoints];
float lnV[numPoints];
float sumLnV2 = 0; // this will hold Σ(lnV^2)
float sumLnVF = 0; // this will hold Σ(lnF^2)

// Single variables for sums
float sumLnV = 0;
float sumLnF = 0;


float readAverageVoltage(int samples = 50) {
  float total = 0;
  for(int i = 0; i < samples; i++){
    int raw = analogRead(forceSensor1);
    //int raw = analogRead(forceSensor1);

    // exponential moving average
    smoothValue = alpha * raw + (1 - alpha) * smoothValue;

    float V = (smoothValue / float(analogMax)) * VinMax;
    total += V;
    delay(10);
  }
  return total / samples;
}

void setup() {
  Serial.begin(9600);
  analogWrite(threevolts, 153) ; //3 volts output 

  // Initialize readings arrays
  for (int i = 0; i < sampleCount; i++) {
    readings[i] = 0;
    readings1[i] = 0;
  }

  pinMode(forceSensor1, INPUT);
  pinMode(forceSensor2, INPUT);
  pinMode(threevolts, OUTPUT); 
  
  analogWrite(threevolts, 153) ; //3 volts output 



  // Compute ln(voltages) and sum
  sumLnV = 0; // reset sum
  for (int i = 0; i < numPoints; i++) {
    lnV[i] = log(voltages[i]);
    sumLnV += lnV[i];
  }

  // Compute ln(forces) and sum
  sumLnF = 0; // reset sum
  for (int i = 0; i < numPoints; i++) {
    lnF[i] = log(forces[i]);
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

  analogWrite(threevolts, 153) ; //3 volts output 
  // ---------------- Sensor 1 ----------------


  float V1 = readAverageVoltage();
  float force = k * pow(V1, n);

  //Serial.print("raw value  ") ;
  //Serial.print(analogRead(forceSensor1)) ; Serial.println("   |v1   ");
  int raw = analogRead(forceSensor1);

  // exponential moving average
  smoothValue = alpha * raw + (1 - alpha) * smoothValue;

  //Serial.print("raw: ");
  //Serial.print(raw);
  //Serial.print(" ");
  Serial.println(smoothValue);
  //Serial.println("");
 // Serial.print(V1);
  //Serial.print("   |k   ");
  //Serial.print(k) ; Serial.print("   |n   "); Serial.print(n); 
  Serial.print("   |force  ");
  Serial.print(force) ;  Serial.print("kg  ");Serial.print(force*9.81) ;  Serial.println("N");
  

  delay(50); // Adjust as needed
}