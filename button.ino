// LED SetUp 
const int LEDblue = 3 ; // when this LED is on, the accelerometer will be calibrating 
const int blueButton = 2 ; 

void setup() {
  Serial.begin(9600); 
  // put your setup code here, to run once:
  pinMode(LEDblue, OUTPUT); 
  pinMode(blueButton, INPUT);

}

void loop() {
  // callibration setup 
  if (digitalRead(blueButton) == HIGH) { //button is pressed 
    Serial.println("PRESSED");
    //callibration() ; // call the calibration code 
    digitalWrite(LEDblue, HIGH);
    //delay(500); 
  }else { 
    Serial.println("UNPRESSED"); 
    digitalWrite(LEDblue, LOW);
  }

}
