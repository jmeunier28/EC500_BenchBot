/*
"""
Open Source Dobot GUI firmware: initial firmware used for testing
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below):
1.
License: MIT
*/



///START STEPPER MOTOR SETTINGS
//The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
int stepperMotorStepsPerRevolution = 200;
//I'm using a ramps 1.4 board with all 3 jumpers connected, which gives me a microstepping mode of 1/16.
//In other words, the motor is set up so it takes 16 steps to move 1 of the default steps.
//microstepping jumper guide for the a4988 stepper driver: https://www.pololu.com/product/1182
int baseMicrosteppingMultiplier = 16;
int upperArmMicrosteppingMultiplier = 16;
int lowerArmMicrosteppingMultiplier = 16;
//The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders. 
//It basically just means that the stepper motor is rotating a smaller gear. That smaller gear is in turn rotating a larger one.
//The gears are set up such that rotating the smaller gear by some number of degrees rotates the larger one by a tenth of that number of degrees (10:1 ratio)
//The bigger gears are actually moving the arm, so the number of steps is increased by a factor of 10 (the gear ratio).
int stepperPlanetaryGearBoxMultiplier = 10;
//This variable will hold the aqctual number of steps per revolution and is calculate by multiplying the three previous variables together.
int baseActualStepsPerRevolution = 0;
int upperArmActualStepsPerRevolution = 0;
int lowerArmActualStepsPerRevolution = 0;
///END STEPPER MOTOR SETTINGS

///START STEPPER MOTOR DRIVER PIN SETTINGS
//I'm using a RAMPS 1.4 board with A4988 stepper motor drivers for each stepper motor on the Dobot.
//There is space for 5 stepper motors on the RAMPS 1.4 board, but only 3 stepper motors are needed to control the Dobot arm.
//Therefore, I'm only actually using 3/5 of the stepper drivers, but I include the pin numbers for all the spots here for completeness.
//Make a note of which stepper driver you are using to control which stepper motor on the Dobot arm.

//X stepper driver spot (I connected this to the upper arm stepper motor)
#define X_STEP_PIN         54//the pin you send a signal to actually step the stepper motor
#define X_DIR_PIN          55//setting this pin to HIGH or LOW dictates in which direction the stepper motor rotates
//unsure what exactly the rest of these do. I think the max and min pins might be related to endstops? This codes is taken from 3D printer code. 
//I'll leave these next 3 lines here for now, but I don't think I will be using them. I would imagine enable either enables or disables the stepper motor.
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

//Y stepper driver spot (I connected this to the lower arm stepper motor)
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

//Z stepper driver spot (has room for two stepper motors, so you can drive simultaneously by the exact same number of steps)
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

//E1 stepper driver, just labeling with E here (I connected this to the base stepper motor)
#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

//E2 stepper driver, just labeling with Q here instead
#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30
///END STEPPER MOTOR DRIVER PIN SETTINGS


//create dobot specific variables to store the relevant pin numbers in
int baseStepPin = E_STEP_PIN;
int baseStepDirPin = E_DIR_PIN;
int upperArmStepPin = X_STEP_PIN;
int upperArmStepDirPin = X_DIR_PIN;
int lowerArmStepPin = Y_STEP_PIN;
int lowerArmStepDirPin = Y_DIR_PIN;







// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 3000;

void setup ()
  {
      
  //START SETUP STEPPER PINS
  //set their modes to output so we can write values to them (1 or 0  aka HIGH or LOW)
  pinMode(X_STEP_PIN  , OUTPUT);
  pinMode(X_DIR_PIN    , OUTPUT);
  pinMode(X_ENABLE_PIN    , OUTPUT);

  pinMode(Y_STEP_PIN  , OUTPUT);
  pinMode(Y_DIR_PIN    , OUTPUT);
  pinMode(Y_ENABLE_PIN    , OUTPUT);

  pinMode(Z_STEP_PIN  , OUTPUT);
  pinMode(Z_DIR_PIN    , OUTPUT);
  pinMode(Z_ENABLE_PIN    , OUTPUT);

  pinMode(E_STEP_PIN  , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN    , OUTPUT);

  pinMode(Q_STEP_PIN  , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN    , OUTPUT);

  //I'm assuming this enables the stepper motors, but not sure
  //again, just copying and pasting from 3D printer software here
  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);
  digitalWrite(Z_ENABLE_PIN    , LOW);
  digitalWrite(E_ENABLE_PIN    , LOW);
  digitalWrite(Q_ENABLE_PIN    , LOW);
  //END SETUP STEPPER PINS


  //START DOBOT SPECIFIC STEPPER MOTOR SETUP
  
  //calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
  baseActualStepsPerRevolution = stepperMotorStepsPerRevolution * baseMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier;
  upperArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * upperArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier;
  lowerArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * lowerArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier;

  //initialize dobot specific variables to store the relevant pin numbers in
  //this will depend on how you have wired up your Dobot's stepper motors to the ramps 1.4 board
  int baseStepPin = E_STEP_PIN;
  int baseStepDirPin = E_DIR_PIN;
  int upperArmStepPin = X_STEP_PIN;
  int upperArmStepDirPin = X_DIR_PIN;
  int lowerArmStepPin = Y_STEP_PIN;
  int lowerArmStepDirPin = Y_DIR_PIN;
  //END DOBOT SPECIFIC STEPPER MOTOR SETUP

  //Connect to the serial port. The input argument is the baud rate. IMPORTNAT: Any software communicating to the arduino must use the same baud rate!
  Serial.begin(115200);
  } // end of setup

// here to process incoming serial data after a terminator received
void process_data (const char * data, int count)
  {
  

//this is where the stepping is actually done. iterates through the step sequence packet that was passed to it and steps when necessary

        //controls the step speed.
        int delayTime = 200;
        
        for(int i=0; i < count; i+=3){
                  //data[0] = baseStep
              if(data[i] == '1'){
                digitalWrite(E_STEP_PIN    , HIGH);
                delayMicroseconds(delayTime);
                digitalWrite(E_STEP_PIN    , LOW);
                delayMicroseconds(delayTime);
                //Serial.println("base step");
              }
              //data[1] = upperStep
              if(data[i+1] == '1'){
                digitalWrite(X_STEP_PIN    , HIGH);
                delayMicroseconds(delayTime);
                digitalWrite(X_STEP_PIN    , LOW);
                delayMicroseconds(delayTime);
              }
              //data[2] = lowerStep
              if(data[i+2] == '1'){
                digitalWrite(Y_STEP_PIN    , HIGH);
                delayMicroseconds(delayTime);
                digitalWrite(Y_STEP_PIN    , LOW);
                delayMicroseconds(delayTime);
              }

        }


      Serial.write("done");

  
  
  }  // end of process_data
  
void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  static int directionDataCounter = 0;

  static int sizeCounter = 0;


  if (inByte == '1' || inByte == '0'){
    // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1)){
        input_line [input_pos++] = inByte;
        sizeCounter++;
      }
      else{
        input_line [input_pos++] = inByte;
        sizeCounter++;
        // buffer limit reached! process input_line here ...
        process_data (input_line,input_pos);
      //Serial.println(sizeCounter);
        // reset buffer for next time
        input_pos = 0;  
        
      }
  }
  else{
    switch (inByte)
      {
  
      case 'e':   // end of text
        input_line [input_pos] = 0;  // terminating null byte
  
        //there should be no more data to process since data is sent and processed in 3-tuples
        // terminator reached! process input_line here ...
        process_data (input_line, input_pos);
        
        // reset buffer for next time
        input_pos = 0; 
         //Serial.println("Line data transfer complete");
         directionDataCounter = 0;
         //Serial.println(sizeCounter);
        break;
  
  
      //set the appropriate directions
      case 'H':
        if(directionDataCounter == 0){
          digitalWrite(E_DIR_PIN, HIGH);
        }
        else if (directionDataCounter == 1){
          digitalWrite(X_DIR_PIN, HIGH);
        }
        else{
          digitalWrite(Y_DIR_PIN, HIGH);
        }
        directionDataCounter++;
        break;
      case 'L':
        if(directionDataCounter == 0){
          digitalWrite(E_DIR_PIN, LOW);
        }
        else if (directionDataCounter == 1){
          digitalWrite(X_DIR_PIN, LOW);
        }
        else{
          digitalWrite(Y_DIR_PIN, LOW);
        }
        directionDataCounter++;
        break;
  
      
      //discard line header data
      case 'l':
      case 's':
        break;
  
      }  // end of switch
  }
   
} // end of processIncomingByte  

void loop()
  {
  // if serial data available, process it
  while (Serial.available () > 0){
    processIncomingByte (Serial.read ());
  }
    
  // do other stuff here like testing digital input (button presses) ...

  }  // end of loop
