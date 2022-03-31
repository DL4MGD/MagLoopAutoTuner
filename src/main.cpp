/*
Version: 1.75 (Final)
Magloop Automatic Controller-Firmware
Arduino Mega 2560 and A4988 Stepper Driver
Author: Michael Poschner (DL4MGD)
Licence: Ablolutely Free
Using MobaTools for steppercontrol
Using Subroutining

          *******************************************************************
          *  Hardwareprogramming is ALWAYS a risky business. Code errors    *
          *  may cause direct hardware DAMAGE. Especially with moving parts *
          *  like stepper motors and switches etc. So check your pinning    *
          *  and your wirering carefully before using this code out of the  *
          *  box.       SO USE THIS FREE CODE FULLY AT YOUR OWN RISK!       *
          *******************************************************************

My Loop-Parameters: Min. capacity = 9.568 MHz
                    Max. capacity = 29,900 MHz

*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MobaTools.h>
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
  const int pinPTT = A12;               // Manually go on air
  const int pinCalTune = A13;            // highest frequency
  const int pinPosSetZero = A14;        // Mobatools set zero
  const int pinManuCal = A15;           // Undefined jet
  const int pinEndSensor = A5;          // Zero detector
  const int pinRelais0 = A4;            // Reilais 0
  const int pinRelais1 = A3;            // Reilais 1
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
  int valPTT = 0;                       // Maximal Capacity 
  int valCalTune = 0;                    // Minimal Capacity
  int valPosSetZero = 0;                // Mobatools set zero
  int valManuCal = 0;                   // Manual recalibrate
  float valREFPO = 0;                   // Initialize reflected power
  float valFWDPO = 0;                   // Initialize forward power
  float valVSWR = 0;                    // VSWR
  float valREFPObef = 0;                // Reflected power value before a tuning step
  float valREFPOaft = 0;                // Refrectec power value after a tuning step
  int CompFwRw = 0;                     // Compare save value once
  int valSpeedSteps = 0;                //
  int valCurrentSpeed = 0;              // Save current speed setting
  int SpeedStepsFast = 25000;           // Fast stepper turning
  int SpeedStepsSlow = 200;             // Slow stepper turning
  int SpeedStepsTuneFast = 7000;        // Beginn tuning with this speed
  int SpeedStepsTuneSlow = 150;         // Finetuning
  int SfZe = 0;                         // Steps away from Zero position
  int RampLen = 250;                    // Smoothing
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
  pinMode(pinPTT, INPUT_PULLUP);
  pinMode(pinCalTune, INPUT_PULLUP);
  pinMode(pinPosSetZero, INPUT_PULLUP);
  pinMode(pinManuCal, INPUT_PULLUP);
  pinMode(pinEndSensor, INPUT_PULLUP);
  pinMode(pinRelais0, INPUT_PULLUP);
  digitalWrite(pinRelais0, LOW);
  pinMode(pinRelais1, INPUT_PULLUP);
  digitalWrite(pinRelais1, LOW);
  lcd.init();
  lcd.backlight();
  myStepper.attach( stepPin, dirPin );
        lcd.setCursor(10,3);
        lcd.print("    ");

  lcd.clear();
  myStepper.setZero();
  digitalWrite(enablePin, HIGH);
}

static void FFWD()
{
lcd.clear();
      while (valFFWD == 0){
  digitalWrite(ms1, HIGH);
  digitalWrite(ms2, HIGH);
  digitalWrite(ms3, HIGH);
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
  digitalWrite(ms1, HIGH);
  digitalWrite(ms2, HIGH);
  digitalWrite(ms3, HIGH);
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
static void PTT()
{
  lcd.clear();
  while (valPTT == 0){
        valPTT=digitalRead(pinPTT);
        digitalWrite(pinRelais0, HIGH);
        delay(25);
        digitalWrite(pinRelais1, HIGH);
          valREFPO = analogRead(pinREFPO);
          valFWDPO = analogRead(pinFWDPO);
          valVSWR = (valFWDPO/valREFPO);
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
          lcd.setCursor(13,3);
          lcd.print("On AIR!");
    }
  digitalWrite(pinRelais0, LOW);
  digitalWrite(pinRelais1, LOW);    
  lcd.clear();
}
//******************** Automatic with calibration first START ************************************************
static void CalTune()

{
  lcd.clear();
  digitalWrite(ms1, LOW);            
  digitalWrite(ms2, HIGH);            
  digitalWrite(ms3, LOW);
  myStepper.setZero();
  for (int i; i < 8000; i--){
        while (valManuCal == 0){
        valManuCal=digitalRead(pinManuCal);
        }
        valEndSensor=digitalRead(pinEndSensor);
        digitalWrite(ms1, LOW);            
        digitalWrite(ms2, HIGH);            
        digitalWrite(ms3, LOW);
        myStepper.attachEnable( enablePin, 10, HIGH );
        myStepper.setSpeedSteps(30000);     
        myStepper.writeSteps(6000);       
        lcd.setCursor(0,0);
        lcd.print("Searching zero.....");
        lcd.setCursor(0,1);
        SfZe=myStepper.readSteps();
        valEndSensor=digitalRead(pinEndSensor);
  if (SfZe >= 9999){
          lcd.setCursor(0,3);
          lcd.print("ERROR: No CalSig!");
          delay(2000);
          lcd.clear();
          break;
        }
  else if (valEndSensor != 0){
          myStepper.stop();
          lcd.clear();
          lcd.print("Found: Calibrated!");
          delay(10);
          myStepper.setZero();
          lcd.clear();
          digitalWrite(enablePin, HIGH);
          break;
        }
    }     
lcd.clear();
valREFPO=analogRead(pinREFPO);
valFWDPO=analogRead(pinFWDPO);
digitalWrite(pinRelais0, HIGH);
delay(200);
digitalWrite(pinRelais1, HIGH);
CompFwRw=analogRead(pinREFPO);
valREFPO=analogRead(pinREFPO);
myStepper.setZero();
lcd.setCursor(0,0);
lcd.print("Tuning now...");
// #### Coarse Tuning-Cycle start 
while ( valREFPO > 0 ){


// Move Stepper start
    digitalWrite(ms1, HIGH); 
    digitalWrite(ms2, HIGH); 
    digitalWrite(ms3, HIGH);   
    myStepper.attachEnable( enablePin, 10, HIGH ); 
    myStepper.setSpeedSteps(SpeedStepsTuneFast);
    myStepper.doSteps(3000);
// Move Stepper end

// #### Coarse Tuning-Cycle stop 
// #### FINE TUNING START
  valREFPO=analogRead(pinREFPO);
// Move Stepper start
  if (valREFPO <= 60){
    digitalWrite(ms1, HIGH);
    digitalWrite(ms2, HIGH);
    digitalWrite(ms3, HIGH);
    myStepper.attachEnable( enablePin, 10, HIGH ); 
    myStepper.setSpeedSteps(SpeedStepsSlow);
    myStepper.doSteps(-10000);
// Move Stepper end
      if (valREFPO < 4){
        myStepper.stop();
        digitalWrite(enablePin, HIGH);
        digitalWrite(pinRelais0, LOW);
        digitalWrite(pinRelais1, LOW);
        valREFPO=analogRead(pinREFPO);
        SfZe=myStepper.readSteps();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Vref=");
        lcd.print(valREFPO);
        lcd.setCursor(0,1);
        lcd.print("Position=");
        lcd.print(SfZe);
        lcd.setCursor(0,2);
        lcd.print("Tuned !");
        lcd.setCursor(0,3);
        lcd.print("CHECK SWR!");
        delay(1000);
        lcd.clear();
        break;
// ### FINE TUNING END        
    }
  }
}
//############################ STOP Auto Tuning STOP  ###################################
  lcd.clear();
}
//******************** Automatic with calibration first STOP ************************************************
static void PosSetZero()
{
  lcd.clear();
        while (valPosSetZero == 0){
        valPosSetZero=digitalRead(pinPosSetZero);
        lcd.setCursor(0,0);
        myStepper.setZero();
        lcd.print("New Zero is now set!");
    }
    lcd.clear();
}

static void ManuCal()
{
  lcd.clear();
  digitalWrite(ms1, LOW);            
  digitalWrite(ms2, HIGH);            
  digitalWrite(ms3, LOW);
  digitalWrite(pinRelais0, LOW);
  digitalWrite(pinRelais1, LOW);
  myStepper.setZero();
  for (int i; i < 8000; i--){
        while (valManuCal == 0){
        valManuCal=digitalRead(pinManuCal);
        }
        valEndSensor=digitalRead(pinEndSensor);
        digitalWrite(ms1, LOW);            
        digitalWrite(ms2, HIGH);            
        digitalWrite(ms3, LOW);
        myStepper.attachEnable( enablePin, 10, HIGH );
        myStepper.setSpeedSteps(30000);     
        myStepper.writeSteps(6000);       
        lcd.setCursor(0,0);
        lcd.print("Searching zero.....");
        lcd.setCursor(0,1);
        SfZe=myStepper.readSteps();
        valEndSensor=digitalRead(pinEndSensor);
  if (SfZe >= 9999){
          lcd.setCursor(0,3);
          lcd.print("ERROR: No CalSig!");
          delay(2000);
          lcd.clear();
          break;
        }
  else if (valEndSensor != 0){
          myStepper.stop();
          lcd.clear();
          lcd.print("Found: Calibrated!");
          delay(10);
          myStepper.setZero();
          lcd.clear();
          digitalWrite(enablePin, HIGH);
          break;
        }   
  }
}
//############################# START Auto Tuning START ####################
static void ATSTART()
{
lcd.clear();
valREFPO=analogRead(pinREFPO);
valFWDPO=analogRead(pinFWDPO);
digitalWrite(pinRelais0, HIGH);
delay(200);
digitalWrite(pinRelais1, HIGH);
CompFwRw=analogRead(pinREFPO);
valREFPO=analogRead(pinREFPO);
// myStepper.setZero();
lcd.setCursor(0,0);
lcd.print("Tuning now...");
// #### Coarse Tuning-Cycle start 
while ( valREFPO > 0 ){


// Move Stepper start
    digitalWrite(ms1, HIGH); 
    digitalWrite(ms2, HIGH); 
    digitalWrite(ms3, HIGH);   
    myStepper.attachEnable( enablePin, 10, HIGH ); 
    myStepper.setSpeedSteps(SpeedStepsTuneFast);
    myStepper.doSteps(3000);
// Move Stepper end

// #### Coarse Tuning-Cycle stop 
// #### FINE TUNING START
  valREFPO=analogRead(pinREFPO);
// Move Stepper start
  if (valREFPO <= 60){
    digitalWrite(ms1, HIGH);
    digitalWrite(ms2, HIGH);
    digitalWrite(ms3, HIGH);
    myStepper.attachEnable( enablePin, 10, HIGH ); 
    myStepper.setSpeedSteps(SpeedStepsSlow);
    myStepper.doSteps(-10000);
// Move Stepper end
      if (valREFPO < 4){
        myStepper.stop();
        digitalWrite(enablePin, HIGH);
        digitalWrite(pinRelais0, LOW);
        digitalWrite(pinRelais1, LOW);
        valREFPO=analogRead(pinREFPO);
        SfZe=myStepper.readSteps();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Vref=");
        lcd.print(valREFPO);
        lcd.setCursor(0,1);
        lcd.print("Position=");
        lcd.print(SfZe);
        lcd.setCursor(0,2);
        lcd.print("Tuned !");
        lcd.setCursor(0,3);
        lcd.print("CHECK SWR!");
        delay(1000);
        lcd.clear();
        break;
// ### FINE TUNING END        
    }
  }
}
//############################ STOP Auto Tuning STOP  ###################################
}
static void ATSTOP()
{
}
void loop()
{
      valFFWD = digitalRead(pinFFWD);
      myStepper.stop( );
      if (valFFWD == 0){
        delay(15); 
        FFWD();
      }
    valFRWD = digitalRead(pinFRWD);
      myStepper.stop( );      
      if (valFRWD == 0){
        delay(15);
        FRWD();
      }
    valSFWD = digitalRead(pinSFWD);
      myStepper.stop( );      
      if (valSFWD == 0){ 
          delay(15);
          SFWD();
      }
    valSRWD = digitalRead(pinSRWD);
      myStepper.stop( );      
      if (valSRWD == 0){
        delay(15);        
          SRWD();
      }
    valATSTART = digitalRead(pinATSTART);
      if (valATSTART == 0){
          delay(15);        
          ATSTART();
      }
    valATSTOP = digitalRead(pinATSTOP);
      if (valATSTOP == 0){
          delay(15);        
          ATSTOP();
      }
      valPTT = digitalRead(pinPTT);
      if (valPTT == 0){
        delay(15);
        PTT();
      }
     valCalTune = digitalRead(pinCalTune);
      if (valCalTune == 0){
        delay(15);        
        CalTune();
      }
      valPosSetZero = digitalRead(pinPosSetZero);
      if (valPosSetZero == 0){
        delay(15);        
        PosSetZero();
      }
      valManuCal = digitalRead(pinManuCal);
      if (valManuCal == 0){
        delay(15);        
        ManuCal();
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
digitalWrite(enablePin, HIGH)   ;
}
