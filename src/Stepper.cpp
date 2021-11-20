/*

Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note 

This is the Linked-List-Merged-Box version - undergoing changes to incorporate the linked list


Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note 
*/


//
//see this sheets URL for values related to deceleration used to inform values in this code
// https://docs.google.com/spreadsheets/d/1IBvHXLke9fBvjETHgyCWAYHGpPX9WbbuqvsiXYv9Ix0/edit#gid=0

//verion 6.0 - change the variable in setup too
//DECELVALUE AND NORMALACCELERATION LOOK GOOD
// check the final moveto values as they may need empirical change on testing
// This routine accepts these commands from the ASCOM Driver via USB Serial Cable:
//
//ES# - emergency stop
//SA999# - Slew to azimuth
//SL# - Slew status request
//
// The routine drives the stepper motor to move the Dome
// It acquires the current azimuth via hardware serial from the encoder


#include <Arduino.h>
#include <AccelStepper.h>
#include "linkedList.h"

//#include <Wire.h>

//Forward declarations
void   Emergency_Stop(int azimuth, String mess);
String WhichDirection();
void   WithinFiveDegrees();
int    getCurrentAzimuth();
void   SendToMonitor();
void   PowerOn();
void   PowerOff();

// end declarations


// define the DC power control pin which is used to drive the gate of the solid state relay
#define power_pin             7        
#define MCU_Reset 12

// Define a stepper and the pins it will use

// pin definitions for step, dir and enable

#define   stepPin 11
#define   dirPin  10
// meaningful names for the serial ports
#define Monitor Serial2
#define ASCOM   Serial
#define Encoder Serial1

//CREATE INSTANCE OF STEPPER MOTOR

AccelStepper  stepper(AccelStepper::DRIVER, stepPin, dirPin, true);

String  receivedData;
boolean DoTheDeceleration;
boolean SlewStatus;             // controls whether the stepper is stepped in the main loop
float   StepsPerSecond;         // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25

boolean TargetChanged = false;

float   normalAcceleration;                            // was incorrectly set to data type int

int     stepsToTarget               =0;
int     DecelValue                  = 800;                // set after empirical test Oct 2020
int     EncoderReplyCounter         = 0;
int     savedAzimuth                = 0;
long    pkstart                     = 0.0l;              // note i after 0.0 denotes long number - same type as millis()



String  TargetMessage = "";
String  QueryDir      ="No Direction";
String  movementstate = "Not Moving";
String  pkversion     = "6.0";


/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/


void setup()
{

  pinMode (MCU_Reset, OUTPUT);                  // used to software reset the stepper MCU from the monitor program
  digitalWrite(MCU_Reset, HIGH);
  pinMode(9, INPUT_PULLUP);                     // see the notes in github. this pulls up the serial Rx pin to 5v.

  stepper.stop();                               // set initial state as stopped

  // Change below to suit the stepper

  SlewStatus         = false;
  StepsPerSecond     = 300.0;                   // changed following empirical testing Oct 2020
  normalAcceleration = 140.0;                   // changed following empirical testing October 17th 2020 - changed from 40 to 20 for trial
  stepper.setMaxSpeed(StepsPerSecond);          // steps per second see below -
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(normalAcceleration);  // steps per second per second.
  // Note V= acceleration * time, so a vlue of e.g. 1 step /s/s takes 10 secs to reach maxspeed of 10 or 15 secs to reach maxspeed 15 etc
  //
  
  // initialise

  CurrentAzimuth    = 0;
  DoTheDeceleration = true;      // used to set deceleration towards target azimuth
  pkstart           = millis();

  delay(2000);                      //why? no original comment is unhelpful

  ASCOM.begin(19200) ;                        // start serial ports ASCOM driver - usb with PC - rx0 tx0 and updi
  Encoder.begin(19200);                        // Link with the Encoder MCU
  Monitor.begin(19200);                        // serial with the Monitor program
//todo remove these two test lines
//  delay(20000);
//  Serial.println("usual crap before get az");
  TargetAzimuth =  getCurrentAzimuth();        // 
//  todo remove the two test lines below
//  Serial.println("usual crap after get az");
//  delay(5000);


initialiseCDArray();

} // end setup

/*
  \\\\\\\\\\\\\\\\/////////////////////////\\\\\\\\\\\\\\\\\\\\\///////////////
  ////////////////\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\
  ////////////////\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\
  \\\\\\\\\\\\\\\\/////////////////////////\\\\\\\\\\\\\\\\\\\\\///////////////
*/


void loop()
{

  // put your main code here, to run repeatedly, perhaps for eternity if the power holds up....
  if(Monitor.available() >0)
  {
    String monitorReceipt = Monitor.readStringUntil('#');
    if(monitorReceipt.indexOf("reset", 0) > -1) 
    {
      digitalWrite(MCU_Reset, LOW);   // LOW resets the MCU
      delay(1000);
      digitalWrite(MCU_Reset, HIGH);  //  HIGH FOR mcu TO RUN
    }
  }

  if (ASCOM.available() > 0)                              // when serial data arrives from the driver on USB capture it into a string
  {

    receivedData = ASCOM.readStringUntil('#');          // read a string from PC serial port usb


    //*************************************************************************
    //******** code for ES process below **************************************
    //**** example of data sent by driver ES#  **************************
    //*************************************************************************
    //*************************************************************************



    if (receivedData.indexOf("ES", 0) > -1)               // Emergency stop requested from C# driver
    {
      //lcd.clear();
      Emergency_Stop(CurrentAzimuth, "Received ES");
      receivedData = "";
    }                                                   // end Emergency Stop



    //*************************************************************************
    //******** code for SA process below **************************************
    //**** example of data sent by driver SA220.00#  **************************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("SA", 0) > -1) //
    {
      
      PowerOn();                   //turn on the power supply for the stepper motor
      // strip off 1st 2 chars
      receivedData.remove(0, 2);

      TargetAzimuth = receivedData.toInt();    // store the target az for comparison with current position
      TargetChanged = true;

      //  Serial.println();
      //  Serial.print("in slewto target received ");
      //  Serial.println(TargetAzimuth);

      if (SlewStatus == false)             // only do this if not slewing
      {
        SlewStatus = true;
        stepper.setAcceleration(normalAcceleration);      // set the acceleration
        stepper.setCurrentPosition(0);                    // initialise the stepper position
        QueryDir = WhichDirection();                      // work out which direction of travle is optimum
      //todo remove 2 lines blow
        //ASCOM.print("So the direction is  ");
        //ASCOM.println(QueryDir);

        if (QueryDir == "clockwise")
        {
          stepper.moveTo(100000);                         // positive number means clockwise in accelstepper library

        }

        if (QueryDir == "anticlockwise")
        {
          
          stepper.moveTo(-100000);                      // negative is anticlockwise in accelstepper library

        }
        
        DoTheDeceleration = true;

        receivedData = "";

      }

    }

    //*************************************************************************
    // ******** code for SL process below *************************************
    //**** example of data sent by driver SL#  **************************
    //*************************************************************************
    //

    if (receivedData.indexOf("SL", 0) > -1) //

    {
      
      if (SlewStatus)
      {
        ASCOM.print("Moving#" );
        
      }
      else
      {
        ASCOM.print("Notmoving#");             // sent to ASCOM serial and picked up by the ASCOM driver
        
      }
      receivedData = "";

    }  // end SL case


  }  // end software serial


  WithinFiveDegrees();

  if (SlewStatus)                    // if the slew status is true, run the stepper
    {

      stepper.run();

      //update the LCD info
      //
      if (  (millis() - pkstart) > 1000.0  )                // one second checks for azimuth value as the dome moves
        {

          SendToMonitor();

          pkstart = millis();

        }

    }


  if (    abs( stepper.distanceToGo() ) < 20   )
    {
      SlewStatus     = false;                      // used to stop the motor in main loop
      movementstate  = "Stopped.  ";               // for updating the lcdpanel

      // Serial.print("ABS STEPPER distance to go....");
      // Serial.println();
      //update the LCD
      TargetMessage = "Target achieved ";

      SendToMonitor();
     
      PowerOff();                                // power off the stepper now that the target is reached.

    }
  else
    {
      movementstate  = "Moving";              // for updating the lcdpanel
      TargetMessage = "Awaiting Target ";
      stepper.run();
    }



} // end void Loop //////////////////////////////////////////////////////////////////////////////////////////////////////


void Emergency_Stop(int azimuth, String mess)
{

  stepper.stop();
  SlewStatus = false;

  // turn off power to the stepper
  PowerOff();
}


String WhichDirection(){ 
  // this routine decides the shortest direction to go based on the difference betwen current and target azimuth
  // optimises battery use by the motor.


  CurrentAzimuth = getCurrentAzimuth();   // this comes from the encoder 
  //savedAzimuth = CurrentAzimuth;          //save this to work out the distance to go
  int clockwiseSteps = calculateClockwiseSteps();
  int antiClockwiseSteps =  360 - clockwiseSteps;
  //todo remove 4 lines blow
  //ASCOM.print("CLOCKWISE STEPS = ");
  //ASCOM.println(clockwiseSteps);
  //ASCOM.print("ANTI Clockwise Steps = ");
  //ASCOM.println(antiClockwiseSteps);
   if (clockwiseSteps <= antiClockwiseSteps)
  {
    stepsToTarget = clockwiseSteps;       // used to define the number of items in the countdown array
    countDown("clockwise");                          // populate the cdarray with the smaller number of steps
    return "clockwise";
  }
  else{
    stepsToTarget = antiClockwiseSteps;    // used to define the number of items in the countdown array
    countDown("anticlockwise");   //populate the cdarray with the smaller number of steps
    return "anticlockwise";
  }
  
}

void WithinFiveDegrees()
{

  if (DoTheDeceleration)
  {

    CurrentAzimuth =   getCurrentAzimuth();

    if (     (abs(CurrentAzimuth - TargetAzimuth) < 5)    && (TargetChanged == true)   )                       // within 5 degrees of target
    {
      
      DoTheDeceleration = false;
      if (QueryDir == "clockwise")
      {
        // set the moveto position to allow 100 steps more for deceleration  +ve for clockwise -ve for anticclock
        
        stepper.moveTo(stepper.currentPosition() + DecelValue); //FROM MA860H Datasheet @0.225 step angle, it requires 1600 steps per rotation
        //of the stepper drive wheel, so 1000 is 0.6 of a rotation
        
      }

      if (QueryDir == "anticlockwise")
      {
        //  stepper.setCurrentPosition(0);
        stepper.moveTo(stepper.currentPosition() - DecelValue);             // check this by printing out current position is it negative?
        
      }
    }

  }

}

int getCurrentAzimuth()
{
  int az;
  
  boolean validaz = false;

  while (validaz == false)
  {

    Encoder.print("AZ#");          //this is sent to the encoder which is coded to return the azimuth of the dome

    delay(100);  //Some testing showed this line improves reliability of the send /receive cycle
    if (Encoder.available() > 0)                       // when serial data arrives capture it into a string
    {

      String receipt = Encoder.readStringUntil('#');   // read a string from the encoder
     
      az = receipt.toInt();                          // convert

      if (  (az > 0) && (az <= 359) )
      {
        validaz = true;
        EncoderReplyCounter ++ ;                       // A counter used to indicate whether the encoder has replied with a valid azimuth
        if (EncoderReplyCounter > 999)                 // reset to zero periodicaly
        {
          EncoderReplyCounter = 0;
        }  //endif
      }  //endif

    }  // endif serial available
  }

  return az;
}   // end getCurrentAzimuth()




void SendToMonitor()
{

  Monitor.print("START#");
  Monitor.print(String(TargetAzimuth)        + '#');
  Monitor.print(movementstate                + '#');
  Monitor.print(QueryDir                     + '#');
  Monitor.print(TargetMessage                + '#');
  CurrentAzimuth= getCurrentAzimuth();
  Monitor.print(String(CDArray[CurrentAzimuth])  + '#');       // in the monitor program this is called distance to target
    
  Monitor.print(String(EncoderReplyCounter)  + '#');
  /*
  list of data need by the monitor program
  targetazimuth
  movementstate
  querydir
  targetmessage
  distance to target
  encoderreplycounter
  */
}


//---------------------------------------------------------------------------------------------------------------

void PowerOn()                          // set the power SSR gate high
{
digitalWrite(power_pin,      HIGH);

delay(2000);                            // gives time for the MA860H unit to power on and stabilise
}

//---------------------------------------------------------------------------------------------------------------

void PowerOff()                         // set the power SSR gate low
{
digitalWrite(power_pin,      LOW);
}
