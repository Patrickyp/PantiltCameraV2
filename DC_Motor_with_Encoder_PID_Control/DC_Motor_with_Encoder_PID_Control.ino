/* Example program to run DC Motor with Driver.

   Connections:

   Power Supply           Driver            Teensy        DC Motor
   5.5V - 35V               VMS
   GND                      GND             GND
                            ENA             DPin 10
                            IN1             DPin 9
                            IN2             DPin 8
                            5V              Vin
                            MOTORA                          Pin
                            MOTORB                          Pin

   Helpful websites:
   http://www.instructables.com/id/Control-DC-and-stepper-motors-with-L298N-Dual-Moto/?ALLSTEPS
   https://www.youtube.com/watch?v=tqfbH429NcE
*/


#include <ADC.h>
//#include <PID_v1.h>

/***************************** Setup Motor drive pins *************************************/
// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;
// define PID coefficients
double P_Coeff = 50;
double I_Coeff = 0;
double D_Coeff = 0;

/***************************** Setup ADC *************************************/
const int readPin1 = A9;       // assign pin A9 as an analog input (it uses ADC0)
const int readPin2 = A8;      // assign pin A8 as an analog input (it uses ADC0)
ADC *adc = new ADC();         // Creat a new adc object;
double AngleAcceptableError = 5; 
int showOutput = 0; //1:True 0:False
void setup()   {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  setupADC();
  Serial.begin(115200);
  delay(1000);}






//////////////////////////////////////////////////////////////
//This is the main loop
/////////////////////////////////////////////////////////////
void loop() {
    /*
     Serial.println("************************** Move Motor Clockwise ************************************");   
     //moveMotorPosition(1,40);    //  moveMotorPosition(motor, angle) 
     moveMotorPosition(2,130);
     delay(2000);
     //Serial.println("************************** Move Motor Counter-Clockwise ****************************");   
     //moveMotorPosition(1,320);
     moveMotorPosition(2,230);
     delay(2000); 
     */
     if (Serial.available()) {
      int motorNumber = Serial.parseInt(); // 1 or 2 for pan or tilt
      int parameter1 = Serial.parseInt(); // 1 or -1 to specify direction to move x degrees, 0 for moving to specific degree.
      int parameter2 = Serial.parseInt(); // number of degrees (distance) to move, or specified degree (position) to move to (depending on previous parameter).
      Serial.clear();
      if (parameter1 == 0){
          moveMotorPosition(motorNumber, parameter2);
          //Serial.clear();
      } else {
          moveMotorDistance(motorNumber, parameter1, parameter2);
      }
    }
}










///////////////////////////////////////////////////////////////////////////////////////////////
//      All Functions are below ...
///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Move particular motor to specified angle (degree).
// Function Inputs:     motor number (1 or 2),   angle to move to (degrees 0-360) ).
/////////////////////////////////////////////////////////////////////////////////////////////
void moveMotorPosition(int motorNumber, double DesiredAngle)    {
  int motoren, motorin1, motorin2;
  if (motorNumber == 1)    {
    //Serial.println("motor1");  
    motoren = enA;
    motorin1 = in1;
    motorin2 = in2;}
  else if (motorNumber == 2)  {
    //Serial.println("motor2");
    motoren = enB;
    motorin1 = in3;
    motorin2 = in4;}
  else  {
    Serial.print("Invalid motorNumber specified for moveMotor: ");
    Serial.println(motorNumber);
    return;    }
  ///////// this is an infinate loop until a break happens  ///////////
  while (true) {           
    double CurrentAngle = getPosition(motorNumber);          // this is returned as an angle (degrees)
    double pid = calcPID(CurrentAngle, DesiredAngle, P_Coeff);
    if (showOutput) {
      Serial.print("PID:");
      Serial.println(pid);
      Serial.print("current angle:");
      Serial.println(CurrentAngle);
      Serial.print("desired position:");
      Serial.println(DesiredAngle);}
    // If Angle is close to desired angle then call it good.
    if ((CurrentAngle < DesiredAngle+AngleAcceptableError) && (CurrentAngle > DesiredAngle-AngleAcceptableError)){
       stopMotor();
       //Serial.print("At (or close to) the desired angle ... ");
       break; }   // exit the infinate loop   
    else if ( CurrentAngle >=  DesiredAngle+AngleAcceptableError)   {
      pid *= -1; // change sign to positive     
      digitalWrite(motorin1, LOW);
      digitalWrite(motorin2, HIGH);
      analogWrite(motoren, pid);}
    else if (CurrentAngle <=  DesiredAngle-AngleAcceptableError)   {
      digitalWrite(motorin1, HIGH);
      digitalWrite(motorin2, LOW);
      analogWrite(motoren, pid);}     }     
  //////////////////////////////////////////////////////////////
  delay(100);
  double pos = getPosition(motorNumber);
  Serial.println(pos);    
}





/* /////////////////////////////////////////////////////////////////////////////////////////
   Function getPosition
   Get the current position of the potentiometer and returns the result in degrees (0-360).
   This function averages 1000 readings and removes outliers.
*//////////////////////////////////////////////////////////////////////////////////////////////
  double getPosition(int motorNumber) {
      double analogVal;
      int pinNumber;
      if (motorNumber == 1) 
          {pinNumber = readPin1;} 
      else if (motorNumber == 2) 
          {pinNumber = readPin2;}
      else 
          { Serial.print("Invalid motor number for getPosition:");
          Serial.println(motorNumber); }
      int numToAverage = 1000;
      double val2[1000];
      double diffSum;
      double average;
      double average2;
      // Read analog values and average them.
      double sum = 0.0;
      for (int x = 0; x < numToAverage; x++) 
        {int val = adc->analogRead(pinNumber); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
        analogVal = (double)val;
        sum += analogVal;                     // save sum
        val2[x] = analogVal; }                // also save individual values
      average = sum / (double)numToAverage;
      // Remove outliers
      double sum2 = 0.0;
      int count = 0;
      for (int x = 0; x < numToAverage; x++) 
        {diffSum = val2[x] - average;
        if (abs(diffSum) < (0.001 * average)) 
            {sum2 += val2[x];
            count++;}}
      average2 = sum2 / (double)count;
      //Serial.print("Average2: ");
      //Serial.println(average2);
      double motorPosition = average2 * 360.0 / 65536.0;                // convert from counts (16 bit) to degrees 
      return motorPosition;    }                                        // this is the value returned



      



void moveMotorDistance(int motorNumber, int dir, double distance) {
  if ((motorNumber != 1) && (motorNumber != 2)){
    Serial.println("Invalid motorNumber for moveMotorDistance.");
    return; }
  else if ((dir != 1) && (dir != -1)){
    Serial.println("Invalid dir for moveMotorDistance.");
    return;  }
  else if ((distance > 330) || (distance < 0)){
    Serial.println("Invalid distance.");
    return;  }
  double pos = getPosition(motorNumber);
  if (dir < 0 ) {
    distance *= -1;  }
  double finalPos = pos + distance;
  if (finalPos < 30) {
    finalPos = 30; }
  else if (finalPos > 330) {
    finalPos = 330;}
  moveMotorPosition(motorNumber, finalPos);
  //Serial.println("1"); 
}

  
  
void stopMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


/* ////////////////////////////////
   Setup ADC settings.
*/ ////////////////////////////////
void setupADC() {
  pinMode(readPin1, INPUT);  // assign pin A9 as an analog input
  pinMode(readPin2, INPUT);  // assign pin A9 as an analog input
  // Set reference voltage to be used for ADC0 (use   ADC_REF_3V3  or  ADC_REF_1V2   or   ADC_REF_EXT).
  adc->setReference(ADC_REF_3V3, ADC_0);        // set ref voltage to 3.3 V
  // Set the number of values to average for ADC_0
  // param num can be 0, 4, 8, 16 or 32
  adc->setAveraging(0); // set number of averages
  // Set the digitization resolution for ADC_0
  adc->setResolution(16); // set bits of resolution
  // Set the Conversion Speed for ADC_0
  // (ADC_VERY_LOW_SPEED  or  ADC_LOW_SPEED  or  ADC_MED_SPEED or  ADC_HIGH_SPEED_16BITS or  ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED)
  adc->setConversionSpeed(ADC_HIGH_SPEED);
  // Set the Sampling Speed for ADC_0
  // (ADC_VERY_LOW_SPEED  or  ADC_LOW_SPEED  or  ADC_MED_SPEED or  ADC_HIGH_SPEED_16BITS or  ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED)
  adc->setSamplingSpeed(ADC_HIGH_SPEED);
}





/*
   Implementation of PID function.
   Inputs are the current and desired anglea (0-360) and a constant k.
   Returns result (0-250).
*/
double calcPID(double current, double desired, double k) {
                        //int minPIDValue = 10;    // Define some const value for the minimum PID
                        //int maxPIDValue = 250;   // Define some const value for the maximum PID
  double diff;
                        //if (current > 1) {
                        //  //Serial.print("greater0 detected");
                        //  diff = desired - current;  }
                        //else if (current <= 1) {
                        //  //Serial.print("0 detected");
                        //  diff = desired - current; }
                        //else {
                          //Serial.print("garbo detected");
                        //  diff = desired;  }
                        //  Serial.print("desired = ");
                        //  Serial.print(desired);
                        //  Serial.print(" ,current = ");
                        //  Serial.print(current,4);
                        //  Serial.print(",diff = ");
                        //  Serial.println(diff);
  diff = desired - current;   // calc proportional diff 
  diff = diff*k;              // apply coefficient
  double out = 255 * diff / 360;
                        //Serial.println(out, 6);
                        // Set result to minimum of 50 otherwise motor won't move.
                        //if ( (result < minPIDValue) && (result > 0)) {
                        //  result = minPIDValue;
                        //}
                        //else if ( (result > -minPIDValue) && (result < 0)) {
                        //  result = -minPIDValue;
                        //}
                        // Limit max value to 250 since that's the max value for analogWrite.
                        //else if (result < -maxPIDValue) {
                        //  result = -maxPIDValue;
                        //}
                        //else if (result > maxPIDValue) {
                        //  result = maxPIDValue;
                        //}
                        // KEEP at max speed longer
                        //else if (result > 100) {
                        //  result = maxPIDValue;
                        //}
                        //else if (result < -100) {
                        //  result = -maxPIDValue;
                        //}
  return out;  
}


  
