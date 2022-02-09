/*
Version: 0.040
Magloop Automatic Controller-Firmware
Arduino Mega 2560 and A4988 Stepper Driver
Author: Michael Poschner (DL4MGD)
Licence: Ablolutely Free
Using MobaTools for steppercontrol
Using Subroutining

Loop-Parameter: Min. capacity = 9.568 MHz
                Max. capacity = 29,900 MHz

*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MobaTools.h>
// Connections to A4988
  const int dirPin = 2;                 // Direction
  const int stepPin = 3;                // Step
  const int enablePin = 4;              // Chip enable/disable
  const int ms1 = 5;                    // Motorstepping 1
  const int ms2 = 6;                    // Motorstepping 2
  const int ms3 = 7;                    // Motortsepping 3
  const int STEPS_PER_REV = 200;        // Motor steps per rotation
  const int i2c_addr = 0x27;            // Define I2C Address - change if reqiuired
  const int pinFFWD = A6;               // Switch fast forward
  const int pinFRWD = A7;               // Switch fast reward
  const int pinSFWD = A8;               // Switch slow forward
  const int pinSRWD = A9;               // Switch slow reward
  const int pinATSTART = A10;           // Autotune start
  const int pinATSTOP = A11;            // Autotune stop
  const int pinMaxIN = A12;             // Lowest frequency
  const int pinMaxOUT = A13;            // highest frequency
  const int pinPosSetZero = A14;        // Mobatools set zero
  const int pinSpare = A15;             // Undefined jet
  const int pinEndSensor = A5;          // Zero detector
MoToStepper myStepper ( 200, STEPDIR );
LiquidCrystal_I2C lcd(0x27,20,4);
  int pinREFPO = A1;                    // Pin, to read reflected power
  int pinFWDPO = A0;                    // Pin, to read forward power 
  int val = 0;                          // Variable, to store analog value of A3
  int valFFWD = 0;                      // Manual fast forward
  int valFRWD = 0;                      // Manual fast reward
  int valSFWD = 0;                      // Manual slow forward
  int valSRWD = 0;                      // Manual slow reward
  int valATSTART = 0;                   // Manual start autotune
  int valATSTOP = 0;                    // Manual stop autotune
  int valMaxIN = 0;                     // Maximal Capacity 
  int valMaxOUT = 0;                    // Minimal Capacity
  int valPosSetZero = 0;                // Mobatools set zero
  int valSpare = 0;                     // Undefined jet
  int valREFPO = 0;                     // Initialize reflected power
  int valFWDPO = 0;                     // Initialize forward power
  int valVSWR = 0;                      // VSWR
  int valREFPObef = 0;                  // Reflected power value before a tuning step
  int valREFPOaft = 0;                  // Refrectec power value after a tuning step
  int valSpeedSteps = 0;                //
  int valCurrentSpped = 0;              // Save current speed setting
  int SpeedStepsFast = 5000;            // Fast stepper turning
  int SpeedStepsSlow = 250;             // Slow stepper turning
  int SfZe = 0;                         // Steps away from Zero position
  int RampLen = 50;                     // Smoothing
  int valEndSensor = 0;                 // Calibrate zero position 

void setup() {
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);                   // Setup serial output speed
  digitalWrite(ms1, LOW);               // Stepping 1 A4988
  digitalWrite(ms2, HIGH);              // Stepping 2 A4988
  digitalWrite(ms3, LOW);               // Stepping 3 A4988
  pinMode(pinFFWD, INPUT_PULLUP);
  pinMode(pinFRWD, INPUT_PULLUP);
  pinMode(pinSFWD, INPUT_PULLUP);
  pinMode(pinSRWD, INPUT_PULLUP);
  pinMode(pinATSTART, INPUT_PULLUP);
  pinMode(pinATSTOP, INPUT_PULLUP);
  pinMode(pinMaxIN, INPUT_PULLUP);
  pinMode(pinMaxOUT, INPUT_PULLUP);
  pinMode(pinPosSetZero, INPUT_PULLUP);
  pinMode(pinSpare, INPUT_PULLUP);
  pinMode(pinEndSensor, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  myStepper.attach( stepPin, dirPin );
  myStepper.setSpeed(1000);
  myStepper.writeSteps(100);
  delay(250);
  myStepper.writeSteps(-100);
  delay(250);
  myStepper.stop();
  myStepper.setZero();


//Calibrate zero position

   lcd.clear();
  while (valEndSensor == 0){
        valEndSensor=digitalRead(pinEndSensor);
        digitalWrite(ms1, LOW);            
        digitalWrite(ms2, HIGH);            
        digitalWrite(ms3, LOW);
        myStepper.attachEnable( enablePin, 10, HIGH );
        myStepper.setRampLen(RampLen);
        myStepper.setSpeedSteps(36000);     
        myStepper.writeSteps(6000);       
        lcd.setCursor(0,0);
        lcd.print("Searching zero......");
        lcd.setCursor(0,1);
        SfZe=myStepper.readSteps();
        if (SfZe >= 5900){
          lcd.setCursor(0,3);
          lcd.print("ERR: Miss CalSig!");
        }
          else if (valEndSensor != 0){
          myStepper.stop();
          lcd.clear();
          lcd.print("Found: Calibrated!");
          delay(2000);
         }
        lcd.setCursor(0,3);
  }
  lcd.clear();
  myStepper.setZero();
}

    

static void FFWD()
{
lcd.clear();   
      while (valFFWD == 0){
  digitalWrite(ms1, LOW);            
  digitalWrite(ms2, HIGH);            
  digitalWrite(ms3, LOW);             
        valFFWD = digitalRead(pinFFWD);
        myStepper.attachEnable( enablePin, 10, HIGH ); 
        myStepper.setSpeedSteps(SpeedStepsFast);
        myStepper.doSteps(10000);
          valREFPO = analogRead(pinREFPO);
          valFWDPO = analogRead(pinFWDPO);
          valVSWR = (valFWDPO/valREFPO);        
          valFFWD = digitalRead(pinFFWD);
          valFRWD = digitalRead(pinFRWD);
          valSFWD = digitalRead(pinSFWD);
          valSRWD = digitalRead(pinSRWD);
          lcd.setCursor(0,0);
          lcd.print("Vref=");
          lcd.print(valREFPO);
          lcd.print("      ");
          lcd.setCursor(0,1);
          lcd.print("Vfwd=");
          lcd.print(valFWDPO);
          lcd.print("      ");
          lcd.setCursor(0,2);
          lcd.print("VSWR=");
          lcd.print(valVSWR+1);
          lcd.print("      ");
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,3);
        lcd.print("Position:");
        lcd.setCursor(10,3);
        lcd.print(SfZe); 
        }
lcd.clear();         
}      

static void FRWD()
{
lcd.clear(); 
      while (valFRWD == 0){
  digitalWrite(ms1, LOW);            
  digitalWrite(ms2, HIGH);            
  digitalWrite(ms3, LOW);                    
        valFRWD = digitalRead(pinFRWD);
        myStepper.attachEnable( enablePin, 10, HIGH ); 
        myStepper.setSpeedSteps(SpeedStepsFast);
        myStepper.doSteps(-10000);
          valREFPO = analogRead(pinREFPO);
          valFWDPO = analogRead(pinFWDPO);
          valVSWR = (valFWDPO/valREFPO);        
          valFFWD = digitalRead(pinFFWD);
          valFRWD = digitalRead(pinFRWD);
          valSFWD = digitalRead(pinSFWD);
          valSRWD = digitalRead(pinSRWD);
          lcd.setCursor(0,0);
          lcd.print("Vref=");
          lcd.print(valREFPO);
          lcd.print("      ");
          lcd.setCursor(0,1);
          lcd.print("Vfwd=");
          lcd.print(valFWDPO);
          lcd.print("      ");
          lcd.setCursor(0,2);
          lcd.print("VSWR=");
          lcd.print(valVSWR+1);
          lcd.print("      "); 
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,3);
        lcd.print("Pos. is:");
        lcd.setCursor(10,3);
        lcd.print(SfZe); 
       }
lcd.clear();        
}       

static void SFWD()
{
lcd.clear(); 
      while (valSFWD == 0){
  digitalWrite(ms1, HIGH);            
  digitalWrite(ms2, HIGH);            
  digitalWrite(ms3, HIGH);                   
        valSFWD = digitalRead(pinSFWD);
        myStepper.attachEnable( enablePin, 10, HIGH ); 
        myStepper.setSpeedSteps(SpeedStepsSlow);
        myStepper.doSteps(10000);
          valREFPO = analogRead(pinREFPO);
          valFWDPO = analogRead(pinFWDPO);
          valVSWR = (valFWDPO/valREFPO);        
          valFFWD = digitalRead(pinFFWD);
          valFRWD = digitalRead(pinFRWD);
          valSFWD = digitalRead(pinSFWD);
          valSRWD = digitalRead(pinSRWD);
          lcd.setCursor(0,0);
          lcd.print("Vref=");
          lcd.print(valREFPO);
          lcd.print("      ");
          lcd.setCursor(0,1);
          lcd.print("Vfwd=");
          lcd.print(valFWDPO);
          lcd.print("      ");
          lcd.setCursor(0,2);
          lcd.print("VSWR=");
          lcd.print(valVSWR+1);
          lcd.print("      ");
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,3);
        lcd.print("Position:");
        lcd.setCursor(10,3);
        lcd.print(SfZe);            
        }
lcd.clear();         
}
static void SRWD()
{
lcd.clear(); 
      while (valSRWD == 0){
  digitalWrite(ms1, HIGH);            
  digitalWrite(ms2, HIGH);            
  digitalWrite(ms3, HIGH);                           
        valSRWD = digitalRead(pinSRWD);
        myStepper.attachEnable( enablePin, 10, HIGH ); 
        myStepper.setSpeedSteps(SpeedStepsSlow);
        myStepper.doSteps(-10000);
          valREFPO = analogRead(pinREFPO);
          valFWDPO = analogRead(pinFWDPO);
          valVSWR = (valFWDPO/valREFPO);        
          valFFWD = digitalRead(pinFFWD);
          valFRWD = digitalRead(pinFRWD);
          valSFWD = digitalRead(pinSFWD);
          valSRWD = digitalRead(pinSRWD);
          lcd.setCursor(0,0);
          lcd.print("Vref=");
          lcd.print(valREFPO);
          lcd.print("      ");
          lcd.setCursor(0,1);
          lcd.print("Vfwd=");
          lcd.print(valFWDPO);
          lcd.print("      ");
          lcd.setCursor(0,2);
          lcd.print("VSWR=");
          lcd.print(valVSWR+1);
          lcd.print("      ");
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,3);
        lcd.print("Position:");
        lcd.setCursor(10,3);
        lcd.print(SfZe);            
        }
}
static void MaxIN()
{
  lcd.clear();
  while (valMaxIN == 0){
        valMaxIN=digitalRead(pinMaxIN);
        digitalWrite(ms1, LOW);            
        digitalWrite(ms2, HIGH);            
        digitalWrite(ms3, LOW);
        myStepper.attachEnable( enablePin, 10, HIGH );
        myStepper.setSpeedSteps(36000);     
        myStepper.writeSteps(3000);       
        lcd.setCursor(0,0);
        lcd.print("Going to pos!");
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,1);
        lcd.print("Position:");
        lcd.setCursor(10,1);
        lcd.print(SfZe);
    }
  lcd.clear();
}
static void MaxOUT()
{
  lcd.clear();
  while (valMaxOUT == 0){
        valMaxOUT=digitalRead(pinMaxOUT);
        digitalWrite(ms1, LOW);            
        digitalWrite(ms2, HIGH);            
        digitalWrite(ms3, LOW);
        myStepper.attachEnable( enablePin, 10, HIGH );
        myStepper.setSpeedSteps(36000);     
        myStepper.writeSteps(1500);       
        lcd.setCursor(0,0);
        lcd.print("Going to pos!");
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,1);
        lcd.print("Position:");
        lcd.setCursor(10,1);
        lcd.print(SfZe);
    }
  lcd.clear();
}

static void PosSetZero()
{
  lcd.clear();
        while (valPosSetZero == 0){
        valPosSetZero=digitalRead(pinPosSetZero);
        lcd.setCursor(0,0);
        myStepper.setZero();
        lcd.print("New Zero is set now!");
    }
    lcd.clear();
}

static void Spare()
{
   lcd.clear();
    digitalWrite(ms1, HIGH);            
    digitalWrite(ms2, HIGH);            
    digitalWrite(ms3, HIGH);              
        while (valSpare == 0){
        valSpare=digitalRead(pinSpare);
        lcd.setCursor(0,0);
        lcd.print("Information!");
        SfZe = myStepper.readSteps();
        lcd.setCursor(0,2);
        lcd.print("Steps from 0:");
        lcd.setCursor(14,2);
        lcd.print(SfZe);
        myStepper.writeSteps(SfZe);

    }
    lcd.clear();
}

//############################# START Auto Tuning START ####################
static void ATSTART()
{
  lcd.clear();
  for( int i=200; i >= 0; i--){
    valATSTOP=digitalRead(pinATSTOP);
    if (valATSTOP ==0 ){
      myStepper.stop();
      break;
        }
    digitalWrite(ms1, LOW);            
    digitalWrite(ms2, HIGH);            
    digitalWrite(ms3, LOW);             
    myStepper.attachEnable( enablePin, 10, HIGH ); 
    myStepper.setSpeedSteps(SpeedStepsFast);
    myStepper.doSteps(10000);
      valREFPObef=analogRead(pinREFPO);
              lcd.setCursor(0,0);
              lcd.print("Tuning...!");
              lcd.setCursor(0,1);
              lcd.print("Reflected:");
              lcd.setCursor(12,1);
              lcd.print(valREFPObef);
              lcd.setCursor(10,3);
              lcd.print("    ");
        SfZe=myStepper.readSteps();
        lcd.setCursor(0,3);
        lcd.print("Position:");
        lcd.setCursor(10,3);
        lcd.print(SfZe);               
         }
         lcd.clear();
}
static void ATSTOP()
{
}
//############################# STOP Auto Tuning STOP  ####################

void loop()
{
      valFFWD = digitalRead(pinFFWD);
      myStepper.stop( );
      if (valFFWD == 0){ 
        FFWD();
      }
    valFRWD = digitalRead(pinFRWD);
      myStepper.stop( );      
      if (valFRWD == 0){
        FRWD();
      }
    valSFWD = digitalRead(pinSFWD);
      myStepper.stop( );      
      if (valSFWD == 0){ 
          SFWD();
      }
    valSRWD = digitalRead(pinSRWD);
      myStepper.stop( );      
      if (valSRWD == 0){
          SRWD();
      }
    valATSTART = digitalRead(pinATSTART);
      if (valATSTART == 0){
          ATSTART();
      }
    valATSTOP = digitalRead(pinATSTOP);
      if (valATSTOP == 0){
          ATSTOP();
      }
      valMaxIN = digitalRead(pinMaxIN);
      if (valMaxIN == 0){
        MaxIN();
      }
     valMaxOUT = digitalRead(pinMaxOUT);
      if (valMaxOUT == 0){
        MaxOUT();
      }
      valPosSetZero = digitalRead(pinPosSetZero);
      if (valPosSetZero == 0){
        PosSetZero();
      }
      valSpare = digitalRead(pinSpare);
      if (valSpare == 0){
        Spare();
      }


lcd.setCursor(0,0);
valREFPO = analogRead(pinREFPO);
valFWDPO = analogRead(pinFWDPO);
valVSWR = (valFWDPO/valREFPO);        
valFFWD = digitalRead(pinFFWD);
valFRWD = digitalRead(pinFRWD);
valSFWD = digitalRead(pinSFWD);
valSRWD = digitalRead(pinSRWD);
SfZe=myStepper.readSteps();
lcd.setCursor(0,0);
lcd.print("Vref=");
lcd.print(valREFPO);
lcd.print("      ");
lcd.setCursor(0,1);
lcd.print("Vfwd=");
lcd.print(valFWDPO);
lcd.print("      ");
lcd.setCursor(0,2);
lcd.print("VSWR=");
lcd.print(valVSWR+1);
lcd.print("      "); 
  lcd.setCursor(0,3);
  lcd.print("Position:");
  lcd.setCursor(10,3);
  lcd.print(SfZe); 
}
