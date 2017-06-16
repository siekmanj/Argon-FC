#include <Wire.h>
#include <Servo.h>


/* IMU Variables */

const int MPU_ADDR = 0x68;

float gY, gX, gZ, gYdrift, gXdrift, gZdrift;
float aY, aX, aZ, aTotal, temperature;
unsigned long loop_timer, zero_timer, last_blink;

float gyroPitchEstimate, gyroRollEstimate, gyroYawEstimate;
float accPitchEstimate, accRollEstimate, accYawEstimate;
float pitchEstimate, rollEstimate, yawEstimate;


/* RC Receiver Variables */

int receiver_ch1, receiver_ch2, receiver_ch3, receiver_ch4, receiver_ch5, receiver_ch6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6, esc_timer, esc_loop_timer;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;



/* PID controller variables */

float KPp, KIp, KDp, KPr, KIr, KDr, KPy, KIy, KDy, KPt, KIt, KDt;
int esc1, esc2, esc3, esc4;
 
float errorTemp, prevErrorP, prevErrorR, prevErrorY, integralP, integralR, integralY, derivativeP, derivativeR, derivativeY, integralT, derivativeT, prevAltitudeError;
float setPointP, setPointR, setPointY, throttle;
float outputPitch, outputRoll, outputYaw, outputThrottle;
float setPointPitch, setPointRoll, setPointYaw;

boolean centerGyro = true;

float convert_to_degrees = 0;

/* Sonar sensor variables */

unsigned long echoStartTime;
unsigned long durationOfEcho;
unsigned long pulseStartTime;
unsigned long timeOfLastInterrupt;


boolean waitingForEcho = false;
boolean sendingPulse = false;
boolean previousInterruptWasHigh = false;

float distance = 0;
float previousDistance = 0;

/* Piezo speaker variables */

unsigned long timeSinceLastBeep;
int counter = 0;
int delayIndex[4] = {300, 150, 100, 150};


/* Altitude hold variables */

boolean altitudeHold = false;
boolean altitudeHoldInit = false;
boolean hardAbort = false;
float throttleSetpoint;
float lastThrottle;
float altitudeSetpoint;
float altitudeError;
boolean readyForAltPID = false;

/* Gimbal variables */

Servo servoPitch, servoRoll;
int gimbalPitch, gimbalRoll;



void setup() {
  tone(A1, 493);
  delay(150);
  tone(A1, 343);
  delay(150);
  noTone(A1);
  delay(50);
  tone(A1, 343, 150);
  DDRD |= B11110000;                                 //Set ports 4, 5, 6 and 7 to output
  DDRC |= B00001000;                                 //Set analog pin 2 to output
  
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCICR |= (1 << PCIE1);                             // set PCIE1 to enable PCMSK1 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9) to trigger an interrupt
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10) to trigger an interrupt
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11) to trigger an interrupt
  PCMSK0 |= (1 << PCINT4);                           // set PCINT4 (digital input 12) to trigger an interrupt
  PCMSK0 |= (1 << PCINT5);
  PCMSK1 |= (1 << PCINT10);                          //set PCINT11 (analog input 3) to trigger an interrupt


  //PITCH TUNED
  KPp = .118; //.12
  KIp = 0.00085; //0.00075
  KDp = 7.9; //8.0
  
  KPr = .11; //11
  KIr = 0.0002;  //0.0015
  KDr = 6.5; //6.7
  
  KPy = 14;
  KIy = .015;
  KDy = 29;

  KPt = 15;
  KIt = 0.000001;
  KDt = 15;

  gYdrift = 0;
  gXdrift = 0;
  gZdrift = 0;
  gY = 0;
  gX = 0;
  gZ = 0;

  
  /*IMU SETUP*/
  pinMode(13, OUTPUT);
  for(int i = 0; i < 10; i++){
      digitalWrite(13, !digitalRead(13));            
      delay(250);
  }
  Serial.begin(9600);   
  //delay(250);
  Serial.println("ArgonFC V 0.4.2");
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send turn-on command
  Wire.write(0x00);                                                    //Send reset command
  Wire.endTransmission();                                              //End the transmission 
  delay(200);
  
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send accelerometer self-test command
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              //End the transmission
  delay(200);
  
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send gyroscope self-test command
  Wire.write(0x08);                                                    
  Wire.endTransmission();  
  delay(200);
  Serial.println("Gyro setup done.");
  
  Serial.print("Calibrating gyro");
  tone(A1, 131, 200);

  int accuracy = 200;
  for(int i = 0; i < accuracy; i++){
    read_MPU6050();
    Serial.print(".");
    gYdrift += gY;
    gXdrift += gX;
    gZdrift += gZ;
    delay(4);

  }
  Serial.println();
  gYdrift /= accuracy;
  gXdrift /= accuracy;
  gZdrift /= accuracy;
  tone(A1, 165, 200);
  int start = 0;
  for(int i = 0; i < 50; i++){
    PORTD |= B11110000;                              //Set ports 4, 5, 6 and 7 to HIGH.
    delayMicroseconds(1000);                         //Wait 1000us. (sends a PWM pulse of 1000)
    PORTD &= B00001111;                              //Set ports 4, 5, 6 and 7 low.
    delay(4);
                                  
  }
  tone(A1, 196, 200);

  while(receiver_ch3 < 800 || receiver_ch3 > 1040 || receiver_ch4 > 1040){
    start ++;                                        
    PORTD |= B11110000;                              //Set ports 4, 5, 6 and 7 to HIGH.
    delayMicroseconds(1000);                         //Wait 1000us. (sends a PWM pulse of 1000)
    PORTD &= B00001111;                              //Set ports 4, 5, 6 and 7 low.
    delay(3);                                        
    if(start == 50){                                //Basically, all this part does is flash the LED every so often.
      digitalWrite(13, !digitalRead(13));            
      start = 0;   
      Serial.println("Waiting for receiver signal...");  
      tone(A1, 240, 100);
    }
  }
  //delay(250);
  for(int i = 0; i < 10; i++){
      digitalWrite(13, !digitalRead(13));            
      delay(75);
  }
  start = 0;
  pitchEstimate = 0;
  rollEstimate = 0;
  yawEstimate = 0;
  servoPitch.attach(3);
  servoRoll.attach(2);
  timeSinceLastBeep = millis();
  zero_timer = micros;
  
}


void loop() {
  
  
    loop_timer = millis();

    //Heartbeat
    /*
    if(altitudeHold){
      
      if(counter > 3) counter = 0;
      
      if(timeSinceLastBeep + delayIndex[counter] < loop_timer){
        if(counter == 2 || counter == 0){
          int tonePitch = map(distance, 0, 800, 800, 200);
          tone(A1, tonePitch);
        }else 
          noTone(A1);
        counter++;
        timeSinceLastBeep = millis();
      }
    }else{
      noTone(A1);
    }*/

    //Sonar Ping
    if(!waitingForEcho){
        if(!sendingPulse){
          pulseStartTime = micros();
          PORTC |= B00001000;
          sendingPulse = true;
        }else if(sendingPulse && (pulseStartTime + 12 < micros())){
          PORTC &= B00000000;
          sendingPulse = false;
          waitingForEcho = true;
        }
    }

    if(millis()-timeOfLastInterrupt > 20){
      distance = -1;
    }else{
      previousDistance = distance;
      distance = ((double)durationOfEcho / 2) / 29.1;
      distance = (distance + previousDistance)/2;
    }
    
    //Gyro calculations
    read_MPU6050();

    gY -= gYdrift;
    gX -= gXdrift;
    gZ -= gZdrift;
    
    gyroRollEstimate += gY * convert_to_degrees;
    gyroPitchEstimate += gX * convert_to_degrees;
    
    gyroPitchEstimate += gyroRollEstimate * sin(gZ * (convert_to_degrees * (3.142/180)));
    gyroRollEstimate -= gyroPitchEstimate * sin(gZ * (convert_to_degrees * (3.142/180))); 

    aTotal = sqrt((aX*aX)+(aY*aY)+(aZ*aZ));            
    accPitchEstimate = asin((float)aY/aTotal)* 57.2958; 
    accRollEstimate = asin((float)aX/aTotal)* -57.2958; 

    if(centerGyro){ //This is the first loop, and the quadcopter may not be on level ground. So we need to use the accelerometer to center the gyroscope.

      centerGyro = false;
      gyroPitchEstimate = accPitchEstimate;
      gyroRollEstimate = accRollEstimate;
      
    }else{
      
      gyroPitchEstimate = gyroPitchEstimate * 0.9995 + accPitchEstimate * 0.0005;
      gyroRollEstimate = gyroRollEstimate * 0.9995 + accRollEstimate * 0.0005;
    }

    pitchEstimate = pitchEstimate * 0.9 + gyroPitchEstimate * 0.1;
    rollEstimate = rollEstimate * 0.9 + gyroRollEstimate * 0.1; 

    
    //Receiver channel reading
    
    if(receiver_ch5 > 1600 && !hardAbort){
      altitudeHold = true;
    }else if(receiver_ch5 > 1600 && hardAbort){
      altitudeHold = false;
    }else if(receiver_ch5 < 1600){
      if(hardAbort) hardAbort = false;

      altitudeHold = false;
      
    }

    if(receiver_ch4 > 1508) setPointYaw = (1508 - receiver_ch4)/8;
    else if(receiver_ch4 < 1492) setPointYaw = (1492 - receiver_ch4)/8;
    
    throttle = receiver_ch3;
    if(!altitudeHoldInit && altitudeHold){ //Haven't initialized but user wants alt hold
      throttleSetpoint = throttle;
      altitudeSetpoint = distance;
      altitudeHoldInit = true;
      tone(A1, 500);
    }
    if(altitudeHoldInit && altitudeHold){ //Initialized and user still wants alt hold
      if(throttleSetpoint - throttle > 40 || throttleSetpoint - throttle < -40 || distance < 0){ //User moved throttle or sonar was unable to get solid reading, abort 
        hardAbort = true;
        outputThrottle = 0;
        noTone(A1);
      }else{
        calculate_altitude_pid();
      }
    }else if(altitudeHoldInit && !altitudeHold){ //Haven't initialized but alt hold still on - disable!
      altitudeHoldInit = false;
      outputThrottle = 0;
      noTone(A1);

    }

    
    
    
    if(receiver_ch2 > 1508) setPointPitch = (1508 - receiver_ch2)/1.6;
    else if(receiver_ch2 < 1492) setPointPitch = (1492 - receiver_ch2)/1.6;
    
    if(receiver_ch1 > 1508) setPointRoll = (receiver_ch1 - 1508)/1.6;
    else if(receiver_ch1 < 1492) setPointRoll = (receiver_ch1 - 1492)/1.6;

    
    /* CALCULATE PID VALUES */
    calculate_pid();
    

    //ESC 1 = pin 4 (BACK LEFT)
    //ESC 2 = pin 5 (FRONT RIGHT)
    //ESC 3 = pin 6 (FRONT LEFT)
    //ESC 4 = pin 7 (BACK RIGHT)
    
    esc1 = throttle - outputPitch + outputRoll + outputYaw + outputThrottle;
    esc2 = throttle + outputPitch - outputRoll + outputYaw + outputThrottle;
    esc3 = throttle + outputPitch + outputRoll - outputYaw + outputThrottle;
    esc4 = throttle - outputPitch - outputRoll - outputYaw + outputThrottle; 

  
    
    if(throttle < 1020){
      esc1 = 1000;
      esc2 = 1000;
      esc3 = 1000;
      esc4 = 1000;
    }
    while(zero_timer + 4000 > micros()){
      
    }
    
    zero_timer = micros();

    
    PORTD |= B11110000;
    
    timer_channel_1 = esc1 + zero_timer;                       //Determine how long each port should be set to HIGH for. (Add the correction (in units of PWM) to the zero timer)
    timer_channel_2 = esc2 + zero_timer;   
    timer_channel_3 = esc3 + zero_timer;   
    timer_channel_4 = esc4 + zero_timer;     
    
    
    while(PORTD >= 16){                                        //Execute the loop until digital port 4 til 7 is low.
      
       esc_loop_timer = micros();                               //Check the current time.

       
       if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111; //port 4 is set low.
       if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111; //port 5 is set low.
       if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111; //port 6 is set low.
       if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111; //port 7 is set low.

    
    }
    //Gimbal
    if(95 + pitchEstimate > 158) servoPitch.write(158);
    else if(95 + pitchEstimate < 48) servoPitch.write(48);
    else servoPitch.write(80 - pitchEstimate);
    
    servoRoll.write(85+rollEstimate);
    

      
    convert_to_degrees = 1/(1000/(millis()-loop_timer)*65.5);

  
 
}


void read_MPU6050(){
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the address of ACCEL_XOUT (first register we want to read)
  Wire.endTransmission();                                              //End transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes
  
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  
  aX = Wire.read() << 8 | Wire.read();                                  
  aY = Wire.read( )<< 8 | Wire.read();                                  
  aZ = Wire.read() << 8 | Wire.read();                                  
  temperature = Wire.read() << 8 | Wire.read();                         
  gX = Wire.read() << 8 | Wire.read();                                 
  gY = Wire.read() << 8 | Wire.read();                                 
  gZ = Wire.read() << 8 | Wire.read();
                                   
}

void calculate_altitude_pid(){
  altitudeError = altitudeSetpoint - distance;

  integralT += altitudeError;
  derivativeT = (altitudeError - prevAltitudeError);
  prevAltitudeError = altitudeError;
  outputThrottle = KPt * altitudeError + KIt * integralT + KDt * derivativeT;

  if(outputThrottle > 150) outputThrottle = 150;
  if(outputThrottle < -150) outputThrottle = -150;
  
}

void calculate_pid(){
  errorTemp = setPointPitch - (pitchEstimate * 17); //This is the proportional term, which is fairly basic.
  
  integralP += errorTemp; //The integral is just the area under the curve, so all we have to do is keep adding the errors together.
  derivativeP = (errorTemp - prevErrorP); //calculate the rate of change based on this loop's error and last loop's error.
  prevErrorP = errorTemp;
  outputPitch = KPp*errorTemp + KIp*integralP + KDp*derivativeP;
  
  if(outputPitch > 400) outputPitch = 400; //make sure the output stays within a certain range.
  if(outputPitch < -400) outputPitch = -400;
  
  
  errorTemp = setPointRoll - (rollEstimate * 17);
  integralR += errorTemp;
  derivativeR = (errorTemp - prevErrorR);
  prevErrorR = errorTemp;
  outputRoll = KPr*errorTemp + KIr*integralR + KDr*derivativeR;
  if(outputRoll > 400) outputRoll = 400;
  if(outputRoll < -400) outputRoll = -400;
  
  
  if(throttle > 1020){
    errorTemp = (setPointYaw) - gZ * convert_to_degrees * 17;
  }else{
    errorTemp = 0;
    prevErrorY = 0;
    outputYaw = 0;
    outputPitch = 0;
    outputRoll = 0;
    integralP = 0;
    integralR = 0;
    integralY = 0;
    prevErrorP = 0;
    prevErrorY = 0;
    prevErrorR = 0;

    outputPitch = 0;
    outputRoll = 0;
    outputYaw = 0;
  }
  integralY += errorTemp;
  derivativeR = (errorTemp - prevErrorY);
  prevErrorY = errorTemp;
  outputYaw = KPy*errorTemp + KIy*integralY + KDy*derivativeY;
  if(outputYaw > 400) outputYaw = 400;
  if(outputYaw < -400) outputYaw = -400;
  
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_ch1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_ch2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_ch3 = current_time - timer_3;         //Channel 3 is current_time - timer_3
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_ch4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }


  //Channel 5=========================================
  if(PINB & B00010000 ){                                       //Is input 12 high?
    if(last_channel_5 == 0){                                   //Input 12 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_5 to current_time
    }
  }
  else if(last_channel_5 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_ch5 = current_time - timer_5;         //Channel 4 is current_time - timer_4
  }
  //Channel 6=========================================
  if(PINB & B00100000 ){                                       //Is input 13 high?
    if(last_channel_6 == 0){                                   //Input 13 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = current_time;                                  //Set timer_6 to current_time
    }
  }
  else if(last_channel_6 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_ch6 = current_time - timer_6;        
  }
}

ISR(PCINT1_vect){
  timeOfLastInterrupt = millis();
  
  if(PINC & B0000100 ){                                     //Is analog pin 3 high?
    if(!previousInterruptWasHigh){
      echoStartTime = micros();
      previousInterruptWasHigh = true;
    }
  }else{
    durationOfEcho = micros() - echoStartTime;
    waitingForEcho = false;
    previousInterruptWasHigh = false;
    readyForAltPID = true;
  }

}
