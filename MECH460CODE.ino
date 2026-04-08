// ************to add *********
// - reset code related to the e-Stop or switch??? 
// - maybe a separate reset??? 
// - write insturction manual 
// - smooth out all angles from the accelerometer 
// - input height and weight at the beginning??? 

#include <Wire.h>
#include <math.h>

// I2C address for the GY-511 (LSM303DLHC)
#define LSM303_ADDRESS_ACCEL 0x19

//-------------------------------PID control----------------------------------
//medium sized values as everything will be moving slow 
float Kp = 2.0;   // Proportional gain 
float Ki = 0.5;   // Integral gain
float Kd = 0.2;   // Derivative gain

float prevError = 0; 
float integral = 0; 


//---------------dempsters body segments and body properties-------------------
const float d_cm_head = 1.0 ; //proximal 
const float d_weight_head = 0.081; 
const float d_cm_trunk = 0.406 ; // distal 
const float d_weight_trunk = 0.4970 ; 
const float d_cm_arm = 0.47 ; //distal 
const float d_weight_Arm = 0.05; 
const float g = 9.81 ; // m/s^2 

//----------------manually change per user---------------------------------------
const float userWeight = 68.0 ; // kg 
const float userTrunk = 0.6 ; // m 
const float userHead = 0.2 ; // m 

//-----------------calculated values to be used----------------------------------
const float m_torso = d_weight_trunk*userWeight; 
const float cm_length = d_cm_trunk*userTrunk; 
const float head_cm = userHead*d_cm_head;
const float head_mass = userWeight*d_weight_head; //kg
const float arm_weight = userWeight*d_weight_Arm;

//----------------------------suit specs-----------------------------------
const float suitWeight = 5; // kg 
const float wedgeWidth = 0.1 ; // m 


// --------------------------Relays ------------------------------
const int relayInput1 = 6; 
const int relayInput2 = 7; 

// ---------------------force sensors -----------------------------
const int forceSensor1 = A2;
const int forceSensor2 = A3 ; 
const int threeVoltOutput = A0 ; // output three volts from the digital pin 8 
const float Vref = 3.3;             // Voltage applied to inverting input (Vref)
const float Rf = 10000.0;          // Feedback resistor in ohms
const int analogMax = 1023;         // 10-bit ADC
const float VinMax = 3.3;          // Max voltage at Arduino pin
//----force sensor callibrated manual inputs-----
// get these values from the callibration code (forceSensorCallibration.ino)
float slope = 0; //slope 
float intercept = 0; // intercept 


//-----------------slight avg for force sensor---------------------
const int sampleCount = 10;         // Number of samples for moving average
float readings[sampleCount];
int readIndex = 0;
float total = 0;
float average = 0;

// ---------------------accelerometer---------------------
const int accelerSCL = 19; // SCL 
const int accelerSDA = 18;  // SDA
float zeroAngle = 0; // variable to store the calibrated zero angle 
float trunkAngleTarget = 0; // initializing base trunk angle based on the accelerometer reading 

//--------------------- LED RED ---------------------
const int LEDredFWD = 5; 
const int LEDredBACK = 4;
// LED SetUp 
const int LEDblue = 3 ; // when this LED is on, the accelerometer will be calibrating 
const int blueButton = 2 ; 
// maybe add a blue button for calibrations?????????????????? *****************************
// manual steps for calibration: Stand up straight 
// turn the suit on 
// stand still until the blue light goes off 
// lean foward and adjust the straps to be tight 
// stand up and start walking on an incline 

//---------------------BATTERY THERMISTOR ---------------------
const int batt1 = A4; 
const int batt2 = A5; 
const float resistorValue = 10000; // 10 kOhm 
const float safetyTemp = 55 ; // degrees celcius (max operating temp of the Li Ion battery)

//max force before injury per shoulder 
const float forceThreshold = 445; //570.0 ; // N (safety threshold)

//------------------thermistor Readings---------------------
float thermistor(float battery) {
  float thermReadingRaw = analogRead(battery);
  float Vout = thermReadingRaw * (5.0 / 1023.0) ; // voltage devider  
  float thermReading = resistorValue * (5.0 - Vout) / Vout;

  // Convert resistance to temperature using simplified Beta equation
  float T0 = 298.15;       // 25°C in Kelvin
  float B = 3950;          // typical NTC value       
  float tempC = (1 / ( (1/T0) + (1/B)*log(thermReading/resistorValue) ) ) - 273.15;
  return tempC ; 
}

//----------------------actuator control----------------------------------------------
// callable function to move actuator forward 
// check polarity on linear actuator 
void actuatorForward() {
  digitalWrite(relayInput1, LOW); 
  digitalWrite(relayInput2, HIGH);
  digitalWrite(LEDredFWD, HIGH);
  digitalWrite(LEDredBACK, LOW);
}
// callable function to move actuator backwards 
void actuatorBackward(){
  digitalWrite(relayInput1, HIGH); 
  digitalWrite(relayInput2, LOW);

  digitalWrite(LEDredFWD, LOW);
  digitalWrite(LEDredBACK, HIGH);
}
// callable function to stop actuator 
void actuatorStop(){
  digitalWrite(relayInput1, LOW); 
  digitalWrite(relayInput2, LOW);

  digitalWrite(LEDredFWD, LOW);
  digitalWrite(LEDredBACK, LOW);
}

// ------------------------- read raw accelerometer -=-------------------------------
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
// callibrates the accelerometer and returns the 'zero' angle for a straight trunk 
float callibration() { 
  // when the blue button is pressed, wait 2 seconds then begin calibration startup 
  delay(2000); 
  digitalWrite(LEDblue, HIGH);

  float sumAngle = 0.0 ; 
  const int samples = 100 ; 

  //actual calibration 
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

  actuatorForward() ; 
  delay(15000); // extend for 15 seconds to ensure the actuator is all the way up: 
  digitalWrite(LEDblue, LOW); 
  actuatorStop(); 
  // to set up the suit: 
  // lean foward until the torso is at 35 Degrees 
  // the linear actuator should be all the way out (10 mm/s 100mm therfore fwd for 15 seconds)
  // pull the straps forward 

  return zeroAngle; // degrees whats the actual angle of the torso 
}

// --------------------------------angle relative to the 0---------------------------------
float readCurrentAngle() {
  int16_t axRaw, ayRaw, azRaw;
  readAccel(axRaw, ayRaw, azRaw);

  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float angle = atan2(ay, az) * 180.0 / PI;

  return angle - zeroAngle; 
} 


//--------------------------------force sensor readings------------------------
float forceSensor(int forceSensorPick) { //forceSensorPick to choose what sensor you want to input
  // Remove oldest reading from total
  total -= readings[readIndex];
  
  // Read new sensor value
  int rawValue = analogRead(forceSensorPick);
  float Vout = (rawValue / float(analogMax)) * VinMax;
  
  // Add new reading to array and total
  readings[readIndex] = Vout;
  total += Vout;
  
  // Advance index and wrap around
  readIndex++;
  if (readIndex >= sampleCount) { 
    readIndex = 0;
  }
  // Calculate moving average voltage
  average = total / sampleCount;
  
  /*// Calculate Rs using averaged voltage, handle near-zero voltage
  float Rs;
  if (average < 0.01) {
    Rs = 1e9;  // Very large resistance if voltage near zero
  } else {
    Rs = (Rf * Vref) / average;
  }*/
  
  
  //float force = (445/(-1.34)) * average + 445 ; 
  float force = slope * average + intercept; 
  if (force < 0 ) { 
    force = 0 ; 
  }
  // force = aV^2 + bV + c
  delay(100);
  return force; //returns the force in N  
}

//-------------------------force sensor callibration---------------------------
// in another code - forceSensorCallibration

//-----------------------------fault----------------------------
void fault() { 
  digitalWrite(LEDredFWD, LOW);
  digitalWrite(LEDredBACK, LOW);
  delay(500) ;
  digitalWrite(LEDredFWD, HIGH);
  digitalWrite(LEDredBACK, HIGH);
  delay(500) ;
}

void setup() { 
  Serial.begin(9600); 
  Wire.begin();

  //-----------------force sensor initial count avg -----------------------
  for (int i = 0; i < sampleCount; i++) {
    readings[i] = 0;
  }
  

  //------------------for accelerometer--------------------------
  // initialize accelerometer: CTRL_REG1_A = 0x27 (enable X/Y/Z, 10Hz)
  Wire.beginTransmission(LSM303_ADDRESS_ACCEL);
  Wire.write(0x20); // CTRL_REG1_A
  Wire.write(0x27); // normal mode, all axes enabled
  Wire.endTransmission();

  // relay output (x4) 
  pinMode(relayInput1, OUTPUT); 
  pinMode(relayInput2, OUTPUT);

  // force sensors (inputs)
  pinMode(forceSensor1, INPUT); 
  pinMode(forceSensor2, INPUT); 
  pinMode(threeVoltOutput, OUTPUT); 

  // accelerometer (inputs) 
  pinMode(accelerSCL, INPUT); 
  pinMode(accelerSDA, INPUT);  

  // LED RED (Outputs based on linear actuator direction)
  pinMode(LEDredFWD, OUTPUT); 
  pinMode(LEDredBACK, OUTPUT);  
  pinMode(LEDblue, OUTPUT); 
  pinMode(blueButton, INPUT);

  //BATTERY THERMISTOR 
  pinMode(batt1, INPUT); // add labels onto battery pack 
  pinMode(batt2, INPUT); 
}

void loop() {
  // output 3volts, constant 
  analogWrite(threeVoltOutput, 153); //constant output of 3v from pin a0 



  // callibration setup 
  while (digitalRead(blueButton) == HIGH) { //button is pressed stay within this loop 
    //Serial.println("PRESSED");
    callibration() ; // call the calibration code 
    delay(500); 
  }

  
  //----------------------------safety stops ------------------------------------------
  //stop based on force sensor
  if (forceSensor(forceSensor1) >= forceThreshold || forceSensor(forceSensor2) >= forceThreshold ) { // N
    // stops linear actuator 
    actuatorStop() ; 
    // flashes the green LED's indicating a fault  
    while (forceSensor(forceSensor1) >= forceThreshold || forceSensor(forceSensor2) >= forceThreshold) {
      fault() ; 
    }
  delay(100) ; //delay for stability 
  } else {
    // just continue the loop 

  }
  // stop based on thermistor readings 
  if (thermistor(batt1) > safetyTemp || thermistor(batt2) > safetyTemp){
    while (thermistor(batt1) > safetyTemp || thermistor(batt2) > safetyTemp) {
      fault() ; 
    }
    actuatorStop(); //stop actuator from moving  
  }else { 
    //cont loop 
  }
  
  // stop based on trunk angle - if the trunk is more then 40 degrees disable the actuator from moving inward 
  // trunk angle is | 40 < x |
  if (readCurrentAngle() > 40) {
    while (readCurrentAngle() > 40) {
      fault() ; 
    } 
    // stops linear actuator 
    actuatorForward() ; // extend the actuator all the way 
    delay(10000) ; // about 10 seconds make sure its all the way out 
    actuatorStop() ; // stop actuator 
    } else { 
    //continue 
  }  


  // readCurrentAngle 
  // forceSensor(forceSensor1)
  // forceSensor(forceSensor2) 

  //---------------------------------Suit logic--------------------------------------------------

  // based on the math of the person height and weight: 
    // ideal pressure is in this range _______ N and _______ N at ____ degrees 
    // ensure the actuator is between that range at the read angle 
    // if there is too much pressure (or nearing the top), linearActuatorFWD
    // if there is not enough pressure or nearing the bottom, linearActuatorRetract 


  //----------------------------------------------PID Control-------------------------------------------------
  //current Angle 
  float trunkAngle = readCurrentAngle() * PI / 180 ; // current trunk angle of the user in radians 
  // current tension in the rope based on the force readings ************************************************************************************ 
  float avgTensionForce = (forceSensor(forceSensor1) + forceSensor(forceSensor2) )/ 2 ; // taking avg of both sensors (
  

  //control the tension of the suit to make the offsetMoment = momentTarget 
  // angle of the suit straps 
  float theta_rope = atan(wedgeWidth / userTrunk); // radians 
  //moment at the hip ignoring the suit reduction  
  float momentBodyWeight = userWeight*g*sin(trunkAngle) * cm_length + suitWeight*sin(trunkAngle) + (userTrunk + head_cm)*g*sin(trunkAngle)*head_mass + userTrunk*sin(trunkAngle)*g*arm_weight ; 
 //moment taken on by the suit 
  float offsetMoment = avgTensionForce*(cos(theta_rope)*wedgeWidth + sin(theta_rope)*userTrunk) ; 
  //overall moment on the hip 
  float work_on_hip = momentBodyWeight - offsetMoment;

  //goal work on hip: 
  // either: 
      // momentTarget = momentBodyWeight 
      // momentTarget = momentBodyWeight*0.35 (35% reduction of work on the hip) 
  float momentTarget = momentBodyWeight*0.35 ; 


  
  //------------------PID Error----------------------
  float error = momentTarget - offsetMoment ; // goal - actual 
  integral += error; // accululate error over time 
  float derivative = error - prevError ; // rate of change for error 
  float pidOutput = Kp * error + Ki * integral + Kd * derivative ; 
  prevError = error; 
  
  //----------------Actuator Control--------------------------
  // trunk angle is | trunkAngle | force should be aimed at momentTarget| 
  if (pidOutput > 5.0) {// Positive error → increase tension
    actuatorBackward(); 
  } 
  else if (pidOutput < -5.0) {// Negative error → reduce tension
    actuatorForward(); 
  } 
  else { // Small error → hold actuator
    actuatorStop();
  }
 //---------------debugging code----------------------------
  //-------------------------------- Debug prints --------------------------------
  
  Serial.print("TrunkAngle: "); 
  Serial.print(trunkAngle*180/PI);
  Serial.print(" | MomentBody: ");
  Serial.print(momentBodyWeight);
  Serial.print(" | OffsetMoment: "); 
  Serial.print(offsetMoment);
  Serial.print(" | Error: "); 
  Serial.println(error);
  

  delay(50); // ~20 Hz control loop


  

  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------

  /*

  //------------------------PID control WITH NO force sensor -----------------------------------
  // if the force sensors prove unreliable, the following PID control will be used 

  // Read current trunk angle (radians)
  float trunkAngle = readCurrentAngle() * PI / 180.0; // radians 
  float dt = 0.05; // 50 ms loop

  float actuatorSpeed = 0.01; // mm/ms 

  // changing the desired trunk angle based on what the current readings are  
  if (trunkAngle >= 0 && trunkAngle <= 10 ) {
    trunkAngleTarget = 10 ; 
  } else if (trunkAngle >= 10 && trunkAngle <= 20) { 
      trunkAngleTarget = 20 ; 
  } else if (trunkAngle >= 20 && trunkAngle <= 30) { 
      trunkAngleTarget = 30 ; 
  } else if (trunkAngle >= 30 && trunkAngle < 40) { 
      trunkAngleTarget = 40 ; 
  } 

  // ---------------- PID Calculation ----------------
  float error = trunkAngle - trunkAngleTarget;
  integral += error * dt;
  float derivative = (error - prevError) / dt;

  float pidOutput = Kp*error + Ki*integral + Kd*derivative;
  prevError = error;

  // ---------------- Determine actuator run time ----------------
  // base times for angle zones
  float retractTime = 0;
  float extendTime = 0;
  float retractLength = 0 ; 
  float retractPercent = 0 ; // what percent should the the actuator retract based on the users 

  // change the desired retract time based on the three designated sections 
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  // ideally this changes per person has trunk length is different 
  // ___% of trunk length must be retracted 
  // based on a sinlge user's testing: 
        //- the overall change is roughly 9.4% of the overall trunk length 
  // retract ___% at 30+ degrees (the most retraction at the steepest lean bc you need the most support) 
  //desired retraction length: 

  if (trunkAngle > 30) {
    retractPercent = 0.09;  // 5% retraction over 30 degrees
  }
  else if (trunkAngle > 20) {
    retractPercent = 0.07;  // 7% retraction over 30 degrees
  }
  else if (trunkAngle > 10) {
    retractPercent = 0.05;  // 9% retraction over 30 degrees 
  }

  //defining how long in mm based on the percent given and userTrunk 
  retractLength = userTrunk * retractPercent * 1000 ; // retract __% of the users trunk (mm) 
  //retractTime based on the goal length 
  retractTime = retractLength * actuatorSpeed;  // ms (will move 150 mm) 
  // PID scales the correction
  retractTime = retractTime * abs(pidOutput); 
  // ----------------------------------------- Actuator Control -----------------------------------------
  // leaning forward → retract actuator 
  if (pidOutput > 1.0 && retractTime > 0) { // retract the actuator an apropriate amount of time 
    actuatorBackward();
    delay(retractTime);
    actuatorStop();
  }

  // trunk returning upright → extend actuator
  else if (pidOutput < -1.0) {
    extendTime = abs(pidOutput) * 300;
    actuatorForward();
    delay(extendTime);
    actuatorStop();
  }


  // ---------------- Debug ----------------
  
  Serial.print("Trunk Angle: ");
  Serial.print(trunkAngle);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.println(pidOutput);

  delay(50);  // ~20Hz loop */
}

 

