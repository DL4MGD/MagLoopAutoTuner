/*
Version: 1.05
Magloop Automatic Controller-Firmware
Arduino Mega 2560 and A4988 Stepper Driver
Author: Michael Poschner (DL4MGD)
Licence: Ablolutely Free
Using MobaTools for steppercontrol
Using Subroutining

Loop-Parameters: Min. capacity = 9.568 MHz
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
  const int pinMaxOUT = A13;            // highest frequency
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
  int valMaxOUT = 0;                    // Minimal Capacity
  int valPosSetZero = 0;                // Mobatools set zero
  int valManuCal = 0;                   // Manual recalibrate
  int valREFPO = 0;                     // Initialize reflected power
  int valFWDPO = 0;                     // Initialize forward power
  int valVSWR = 0;                      // VSWR
  int valREFPObef = 0;                  // Reflected power value before a tuning step
  int valREFPOaft = 0;                  // Refrectec power value after a tuning step
  int CompFwRw = 0;                     // Compare save value once
  int valSpeedSteps = 0;                //
  int valCurrentSpeed = 0;              // Save current speed setting
  int SpeedStepsFast = 5000;            // Fast stepper turning
  int SpeedStepsSlow = 80;              // Slow stepper turning
  int SpeedStepsTuneFast = 2500;        // Beginn tuning with this speed
  int SpeedStepsTuneSlow = 50;          // Finetuning
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
  pinMode(pinMaxOUT, INPUT_PULLUP);
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
//Calibrate zero position
lcd.clear();
  while (valEndSensor == 0){
        valEndSensor=digitalRead(pinEndSensor);
        digitalWrite(ms1, LOW);            
        digitalWrite(ms2, HIGH);            
        digitalWrite(ms3, LOW);
        myStepper.attachEnable( enablePin, 10, HIGH );
        myStepper.setSpeedSteps(30000);     
        myStepper.writeSteps(6000);       
        lcd.setCursor(0,0);
        lcd.print("Searching zero......");
        lcd.setCursor(0,1);
        SfZe=myStepper.readSteps();
        if (SfZe >= 5900){
          lcd.setCursor(0,3);
          lcd.print("ERROR: No CalSig!");
        }
          else if (valEndSensor != 0){
          myStepper.stop();
          lcd.clear();
          lcd.print("Found: Calibrated!");
          delay(1000);
         }
        lcd.setCursor(0,3);
  }
  lcd.clear();
  myStepper.setZero();
  digitalWrite(enablePin, HIGH);
}
