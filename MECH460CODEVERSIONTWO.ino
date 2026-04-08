// ************to add *********

// - write insturction manual 
// - smooth out all angles from the accelerometer 
// - input height and weight at the beginning??? 

#include <Wire.h>
#include <math.h>

// I2C address for the GY-511 (LSM303DLHC)
#define LSM303_ADDRESS_ACCEL 0x19

//*******MUST UPDATE EACH USER**********
//----------------manually change per user (5 FT 9 IN MALE)---------------------------------------
const float userWeight = 79.0 ; // kg 
const float userTrunk = 480 ; // users trunk length in mm 
const float userHead = 200 ; // bottom of neck to top of head length mm 

//-------------------------------PID control----------------------------------
//medium sized values as everything will be moving slow 
float Kp = 0.6;   // Proportional gain 
float Ki = 0.1;// Integral gain
float Kd = 0.03;   // Derivative gain

float prevError = 0; 
float integral = 0; 
float derivative = 0; // rate of change for error 
float pidOutput = 0 ; 
float pidOutput2 = 0 ; 
// ensuring the PID errors stays within a range 
float maxIntegral = 77.0 ; 
float maxDerivative = 400.0 ; 
float prevDerivative = 0.0; 




//---------------dempsters body segments and body properties-------------------
const float d_cm_head = 1.0 ; //proximal 
const float d_weight_head = 0.081; 
const float d_cm_trunk = 0.406 ; // distal 
const float d_weight_trunk = 0.4970 ; 
const float d_cm_arm = 0.47 ; //distal 
const float d_weight_Arm = 0.05; 
const float g = 9.81 ; // m/s^2 


//-----------------calculated values to be used----------------------------------
const float m_torso = d_weight_trunk*userWeight; 
const float cm_length = d_cm_trunk*userTrunk; 
const float head_cm = userHead*d_cm_head;
const float head_mass = userWeight*d_weight_head; //kg
const float arm_weight = userWeight*d_weight_Arm;

//----------------------------suit specs-----------------------------------
const float suitWeight = 5; // kg 
const float wedgeWidth = 100 ; // mm

// --------------------------Relays ------------------------------
const int relayInput1 = 6; 
const int relayInput2 = 7; 

// -----------------------------Linear Actuator Control---------------------------
float actualLength = 100.0 ; // mm 
float timeCounter = 0.00; // ms 
float maxRetract = 0.094; // max retract percent based on the users trunk length 9.4%/100 

// ---------------------force sensors -----------------------------
const int forceSensor1 = A2;
const int forceSensor2 = A3 ; 
const int threeVoltOutput = A0 ; // output three volts from the digital pin 8 
const float Vref = -3.0;             // Voltage applied to inverting input (Vref)
const float Rf = 10000.0;          // Feedback resistor in ohms
const int analogMax = 1023;         // 10-bit ADC
const float VinMax = 3.0;          // Max voltage at Arduino pin
//----force sensor callibrated manual inputs-----
// get these values from the callibration code (f-sensequadcallibration.ino)
float smoothValue = 0 ; // for smoothing force values 
float alpha = 0.2 ; // smoothing factor (0.05-0.2)

// for the power law equation that best fits the force sensors 
 
float lnk = 67.684135 ; 
float k = exp(lnk);
float n = -86.801200 ; 
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
//const float voltages[numPoints] = {2.2227, 2.2048, 2.1731, 2.1233, 2.1175, 2.1642};
//const float forces[numPoints]   = {0.1, 0.5, 2.3, 6.8, 11.33, 3};

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

// ---------------------accelerometer---------------------
//const int accelerSCL = 19; // SCL 
//const int accelerSDA = 18;  // SDA
float zeroAngle = 0; // variable to store the calibrated zero angle 
float trunkAngleTarget = 0; // initializing base trunk angle based on the accelerometer reading 

bool calibrated = false; // Will be set true only after calibration

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
const int batt1 = A1; 
//const int batt2 = A5; 
const float resistorValue = 10000; // 10 kOhm 
const float safetyTemp = 55 ; // degrees celcius (max operating temp of the Li Ion battery)

//max force before injury per shoulder 
const float forceThreshold = 445; //570.0 ; // N (safety threshold) this can remain constant as 570 N is the max force before injury would occur from sustained use of the suit. 


//------------------thermistor Readings---------------------
float thermistor(float battery) {
  float thermReadingRaw = analogRead(battery);
  float Vout = thermReadingRaw * (5.0 / 1023.0) ; // voltage devider  
  float thermReading = resistorValue * (5.0 - Vout) / Vout;

  // Convert resistance to temperature using simplified Beta equation
  float T0 = 298.15;// 25°C in Kelvin
  float B = 3950;// typical NTC value       
  float tempC = (1 / ( (1/T0) + (1/B)*log(thermReading/resistorValue) ) ) - 273.15;
  return tempC ; 
}

//----------------------actuator control----------------------------------------------
// callable function to move actuator forward 
// check polarity on linear actuator 
void actuatorForward() { // move actuator fwd and turn the green LED Fwd on 
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
  //digitalWrite(LEDblue, HIGH);

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

  delay(2000); 
  //digitalWrite(LEDblue, HIGH);
  actuatorForward() ; 
  delay(9000); // extend for 7 seconds to ensure the actuator is all the way up: 

  actuatorStop();  
  //digitalWrite(LEDblue, LOW); 

  // to set up the suit: 
  // lean foward until the torso is at 35 Degrees 
  // the linear actuator should be all the way out (10 mm/s 100mm therfore fwd for 15 seconds)
  // pull the straps forward 
  delay(2000); // delay another 2 seconds 
  

  return zeroAngle; // degrees whats the actual angle of the torso 

}

// --------------------------------angle relative to the 0---------------------------------
float readCurrentAngle() {
  int16_t axRaw, ayRaw, azRaw;
  readAccel(axRaw, ayRaw, azRaw);

  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float angle = atan2(ay, az) * 180.0 / PI;
  angle -= zeroAngle;
  if (angle > 90) { 
    angle = 0; // if the angle reads over 180 (the participant leans backwards and the angle becomes around 360, dont pick up a large trunk lean, pick up a striaght back)
  }

  return angle; 
} 


//--------------------------------force sensor readings------------------------
float readAverageVoltage(int forceSensorReading) { // forceSensorReaeding changes depending on what sensor you want to call to the function 
  int samples = 50;
  float total = 0;
  for(int i = 0; i < samples; i++){
    int raw = analogRead(forceSensorReading); 
    //int raw = analogRead(forceSensor1);

    // exponential moving average
    smoothValue = alpha * raw + (1 - alpha) * smoothValue;

    float V = (smoothValue / float(analogMax)) * VinMax;
    total += V;
    delay(10);
  }

  return total / samples;
}

float forceSensor(int forceSensor) { // outputs the force of each declared force sensor 
  float V1 = readAverageVoltage(forceSensor);
  float force = k * pow(V1, n);
  return force ; 
}
//-------------------------force sensor callibration---------------------------
// in another code - f-sensequadcallibration.ino 
    //must use 6 weights preferably 0.01kg < x < 20kg+ 

// PID control for the force sensors 
float computePID(float target, float actual, float dt) {
    
    float error = target - actual;

    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral);

    float derivative = (error - prevError) / dt;

    float alpha = 0.1; // smoothing factor
    derivative = alpha * derivative + (1 - alpha) * prevDerivative;
    derivative = constrain(derivative, -maxDerivative, maxDerivative);

    float pidOutput = Kp * error + Ki * integral + Kd * derivative;

    prevError = error;
    prevDerivative = derivative;

    return pidOutput;
}

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
  Wire.setClock(20000); // 20 kHz (lower the accelerometer readings for long thin wires)

  
  

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
  
  //-----------------force sensor initial count avg -----------------------
  for (int i = 0; i < sampleCount; i++) {
    readings[i] = 0;
    readings1[i] = 0;
  }

  // accelerometer (inputs) 
  //pinMode(accelerSCL, INPUT); 
  //pinMode(accelerSDA, INPUT);  

  // LED RED (Outputs based on linear actuator direction)
  pinMode(LEDredFWD, OUTPUT); 
  pinMode(LEDredBACK, OUTPUT);  
  pinMode(LEDblue, OUTPUT); 
  pinMode(blueButton, INPUT);

  //BATTERY THERMISTOR 
  pinMode(batt1, INPUT); // add labels onto battery pack 
  //pinMode(batt2, INPUT); 

  bool calibrated = false;


}

void loop() {
  // output 3volts, constant 
  analogWrite(threeVoltOutput, 153); //constant output of 3v from pin a0 



  // callibration setup 
  if (digitalRead(blueButton) == HIGH && !calibrated) { //button is pressed stay within this loop 
    
    digitalWrite(LEDblue, HIGH); 
    callibration() ; // call the calibration code 
    delay(500); 
    calibrated = true; // indicating the suit has been callibrated 
    digitalWrite(LEDblue, LOW) ; 
    timeCounter = 0 ; // setting the actuator location counter to zero  
    Serial.println("DONE CALLIBRATING");
  } else if (digitalRead(blueButton) == LOW && !calibrated) { //if the button is not pressed, and the suit is not callibrated stop the loop 
    Serial.println("Waiting for calibration..."); //trouble shooting rpint statement 
    actuatorStop();// ensure actuator stays stopped
    while (!calibrated) {
      // flash blue LED indicating the suit needs callibration 
      digitalWrite(LEDblue, HIGH); 
      delay(500); 
      digitalWrite(LEDblue, LOW); 
      delay(500); 
      if (digitalRead(blueButton) == HIGH) {
        Serial.println("Exiting loop");
        break; // exits the while loop immediately  
      }
    } 
    delay(50);
    return;// skip the rest of the loop until calibrated
  }

  //turn blue LED off for the rest of the loop 
  digitalWrite(LEDblue, LOW); 

  //----------------------------safety stops ------------------------------------------
  //stop based on force sensor
  if (forceSensor(forceSensor1) >= forceThreshold || forceSensor(forceSensor2) >= forceThreshold ) { // N
    // stops linear actuator 
    actuatorStop() ; 
    // flashes the green LED's indicating a fault  
    while (forceSensor(forceSensor1) >= forceThreshold || forceSensor(forceSensor2) >= forceThreshold) {
      fault() ; 
    }
  delay(50);
  return;
  calibrated = false;
  } else {
    // just continue the loop 

  } 
  // stop based on thermistor readings 
  if (thermistor(batt1) > safetyTemp ) { // || thermistor(batt2) > safetyTemp){
    while (thermistor(batt1) > safetyTemp ) { // || thermistor(batt2) > safetyTemp) {
      fault() ; 
    }
    actuatorStop(); //stop actuator from moving  
    calibrated = false; 
    return; 
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
    return; 
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
  
  
  
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------

  /*
  
  
  //----------------------------------------------PID Control-------------------------------------------------
  //current Angle 
  float trunkAngle = abs(readCurrentAngle() * PI / 180); // current trunk angle of the user in radians 
  // current tension in the rope based on the force readings ************************************************************************************ 
  
  float avgTensionForce = (forceSensor(forceSensor1) * 9.81) ; //  forceSensor(forceSensor2) )/ 2 ; // taking avg of each sensor  (this is reading N)
  float avgTensionForce2 = (forceSensor(forceSensor2) * 9.81) ; // [N]
  
  //-----------------------------these equations use m, kg, s, N unlike the rest of the code -----------------------------------------------------
  //control the tension of the suit to make the offsetMoment = momentTarget 
  // angle of the suit straps 
  float theta_rope = atan(wedgeWidth / userTrunk); // radians (both wedgeWdith and userTrunk are in mm)
  //moment at the hip ignoring the suit reduction  

  float momentBodyWeight = userWeight*g*sin(trunkAngle) * (cm_length/1000)+ suitWeight*sin(trunkAngle) + ((userTrunk/1000) + (head_cm/1000))*g*sin(trunkAngle)*head_mass + (userTrunk/1000)*sin(trunkAngle)*g*arm_weight ; 
  // opposite moment that is created by the suit (how much moment is taken off the hip)
  float offsetMoment = avgTensionForce*(cos(theta_rope)*(wedgeWidth/1000) + sin(theta_rope)*(userTrunk/1000)) ; 
  float offsetMoment2 = avgTensionForce2*(cos(theta_rope)*(wedgeWidth/1000) + sin(theta_rope)*(userTrunk/1000)) ; 
  //overall moment on the hip 
  float work_on_hip = momentBodyWeight - offsetMoment;
  float work_on_hip2 = momentBodyWeight - offsetMoment2;

  //goal work on hip: 
  // either: 
      // momentTarget = momentBodyWeight 
      // momentTarget = momentBodyWeight*0.35 (35% reduction of work on the hip) 
  float momentTarget = momentBodyWeight*0.35 ; 

  // create a target tension to keep: 
  float rope_tension_target = momentTarget / (cos(theta_rope)*(wedgeWidth/1000)+ sin(theta_rope)*(userTrunk/1000)) ; // [N]
  //max the target tension at the tension with a trunk lean of 40 degrees
  float rope_tension_max = ((userWeight*g*sin(0.610) * (cm_length/1000)+ suitWeight*sin(0.610) + ((userTrunk/1000) + (head_cm/1000))*g*sin(0.610)*head_mass + (userTrunk/1000)*sin(0.610)*g*arm_weight )*0.35 ) / (cos(theta_rope)*(wedgeWidth/1000)+ sin(theta_rope)*(userTrunk/1000)) ; // [N]
  rope_tension_target = constrain(rope_tension_target, 0, rope_tension_max);

  float dt = 0.05; // 50 ms loop

  
  //------------------PID Error----------------------
  // create the error based on the desired rope tension 
  
  
  //float error = rope_tension_target - avgTensionForce ;// momentTarget - work_on_hip ; // goal - actual 
  //integral += error * dt; // accululate error over time 
  //derivative = (error - prevError) /dt ; // rate of change for error 
  //pidOutput = Kp * error + Ki * integral + Kd * derivative ; 
  //prevError = error; 
  

  pidOutput = computePID(rope_tension_target, avgTensionForce, dt);
  pidOutput2 = computePID(rope_tension_target, avgTensionForce2, dt);


  //----------------Actuator Control--------------------------
  // trunk angle is | trunkAngle | force should be aimed at momentTarget|
  //need more tension  
  if (pidOutput > 20.0 || pidOutput2 > 20.0) {// Positive error → increase tension
    actuatorBackward(); 
    Serial.print("BACKWARDS | Angle: "); 
    Serial.print(trunkAngle*180/PI);
    Serial.print(" | target tension : "); 
    Serial.print(rope_tension_target);
    Serial.print(" | rope tension Actual: "); 
    Serial.print(avgTensionForce);
    Serial.print(" N ");Serial.print(avgTensionForce2);
    Serial.print(" | pidOutput: "); 
    Serial.println(pidOutput);

  } 
  else if (pidOutput < -7.0 || pidOutput < -7.0) {// Negative error → reduce tension
    actuatorForward(); 
    Serial.print("BACKWARDS | Angle: "); 
    Serial.print(trunkAngle*180/PI);
    Serial.print(" | target tension : "); 
    Serial.print(rope_tension_target);
    Serial.print(" | rope tension Actual: "); 
    Serial.print(avgTensionForce);
    Serial.print(" N ");Serial.print(avgTensionForce2);
    Serial.print(" | pidOutput: "); 
    Serial.println(pidOutput);
  } 
  else { // Small error → hold actuator
    actuatorStop();
    Serial.print("BACKWARDS | Angle: "); 
    Serial.print(trunkAngle*180/PI);
    Serial.print(" | target tension : "); 
    Serial.print(rope_tension_target);
    Serial.print(" | rope tension Actual: "); 
    Serial.print(avgTensionForce);
    Serial.print(" N ");Serial.print(avgTensionForce2);
    Serial.print(" | pidOutput: "); 
    Serial.println(pidOutput);
  }
 
  

  delay(50); // ~20 Hz control loop 

}
  
*/
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------


  //------------------------PID control WITH NO force sensor -----------------------------------
  // if the force sensors prove unreliable, the following PID control will be used 
  // ---------------- Determine actuator run time ----------------
  // base times for angle zones
  float retractTime = 0;
  float extendTime = 0;
  float retractLength = 0 ; 
  float retractPercent = 0 ; // what percent should the the actuator retract based on the users 

  // Read current trunk angle (radians)
  float trunkAngle = abs(readCurrentAngle() * PI / 180.0); // radians 
  float dt = 0.05; // 50 ms loop

  float actuatorSpeed = 0.01; // mm/ms 

  

  // change the desired retract time based on the three designated sections 
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  // ideally this changes per person has trunk length is different 
  // ___% of trunk length must be retracted 
  // based on a sinlge user's testing: 
        //- the overall change is roughly 9.4% of the overall trunk length 
  // retract ___% at 30+ degrees (the most retraction at the steepest lean bc you need the most support) 
  //desired retraction length:

  // 9% retraction over 30 degrees 
  // 7% retraction over 20 degrees 
  // 5% (of torso) retraction over 10 degrees
  if (trunkAngle > 0.698 ){ // 40 degrees 
    retractPercent = 0.0; // re extend the actuator 
  } 
  else if (trunkAngle > 0.523 ) { // 30 degrees 
    retractPercent = 0.09;  
  }
  else if (trunkAngle > 0.349) { // 20 degrees 
    retractPercent = 0.07;  
  }
  else if (trunkAngle > 0.175) { // 10 degrees 
    retractPercent = 0.05;  
  }else { // 0 degrees
    retractPercent = 0.0; // fully extended do not compress shoulders when standing upright 
  }

  //retractPercent = maxRetract *sin(trunkAngle);
  //retractPercent = constrain(retractPercent, 0, maxRetract);


  // setting the target length based on the given retract percentages 
  float targetLength = 100.0 - (userTrunk*retractPercent); // overall Length - retract Length = desired length (mm) 
  // set the actuator length as fully extended (100mm) at calibration 
  actualLength -= retractTime*actuatorSpeed ; // counts time in the loop to attempt to locate actuator ; this will change based on if the batteries are charged or not 



  
  //taking this out because changing the PID based on actuator length 

  //defining how long in mm based on the percent given and userTrunk 
  //retractLength = userTrunk * retractPercent / 1000 ; // retract __% of the users trunk (mm) 
  //retractTime based on the goal length 
  retractTime = abs(actualLength - targetLength) / actuatorSpeed;  // ms (will move 150 mm) 
  // PID scales the correction
  //retractTime = retractTime * abs(pidOutput); 

    // ---------------- PID Calculation ----------------
  // error based on where the linear actuator is, what is the current position and what is the desired position 
  float error = targetLength - actualLength ; // trunkAngle - trunkAngleTarget;
  integral += error * dt;
  float derivative = (error - prevError) / dt;

  float pidOutput = Kp*error + Ki*integral + Kd*derivative;
  prevError = error;


  // ----------------------------------------- Actuator Control -----------------------------------------
  // actual length is smaller then target length (extend actuator)
  if (pidOutput > 1.0) { // retract the actuator an apropriate amount of time 
    Serial.println("extending");
    actuatorForward();
    delay(retractTime);
    //timeCounter = -retractTime ; 
    actuatorStop();
    actualLength += retractTime * actuatorSpeed;
    actualLength = constrain(actualLength, 0, 100);
    Serial.println("stop");
  }

  // actual length is bigger then target length (retract actuator)
  else if (pidOutput < -1.0) {
    Serial.println("retracting"); 
    actuatorBackward();
    delay(retractTime);
    //timeCounter = retractTime ;
    actuatorStop();
    actualLength -= retractTime * actuatorSpeed;
    actualLength = constrain(actualLength, 0, 100);
    Serial.print("stop"); 
  }


  // ---------------- Debug ----------------
  Serial.print("  |  Trunk Angle: ");
  Serial.print(trunkAngle * 180 / PI);
  Serial.print(" Deg? | Error: ");
  Serial.println(error);
  Serial.print(" | actualLength");
  Serial.print(actualLength);
  Serial.print(" | Desired Length") ;
  Serial.print(targetLength);
  Serial.print("  |  retract percent") ; 
  Serial.print(retractPercent); 
  Serial.print(" | Move Time(ms): ");
  Serial.println(retractTime);


  delay(50);  // ~20Hz loop 

  
}

 
