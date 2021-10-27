/*

Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note 

This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch
This is the Merged-Box-Stepper branch

Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note Note 
*/







// This code is the current software loaded on the Stepper MCU - Last upload 9-9-21 from dev m/c
//USes the new faster LCD Library:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home#!usage
//the above has good setup info for backlight and contrast analogue pins
//
//see this sheets URL for values related to deceleration used to inform values in this code
// https://docs.google.com/spreadsheets/d/1IBvHXLke9fBvjETHgyCWAYHGpPX9WbbuqvsiXYv9Ix0/edit#gid=0

//verion 5.0 - change the variable in setup too
//DECELVALUE AND NORMALACCELERATION LOOK GOOD
// check the final moveto values as they may need empirical change on testing
// This routine accepts these commands from the ASCOM Driver via USB Serial Cable:
//
//ES# - emergency stop
//SA999.99# - Slew to azimuth
//SL# - Slew status request
//
// The routine drives the stepper motor to move the Dome
// It acquires the current azimuth via hardware serial from the encoder


#include <arduino.h>
#include <AccelStepper.h>

#include <Wire.h>

//Forward declarations
void   Emergency_Stop(float azimuth, String mess);
void   lcdprint(int col, int row, String mess);
String WhichDirection();
//todo remove the linebelow which was for testing



void   WithinFiveDegrees();
float  getCurrentAzimuth();
void   UpdateThelcdPanel();
int    AngleMod360();
void   SendToMonitor();
void   PowerOn();
void   PowerOff();
// end declarations


// define the DC power control pin which is used to drive the gate of the solid state relay
#define power_pin             7        

// Define a stepper and the pins it will use

// pin definitions for step, dir and enable

#define                stepPin 11
#define                dirPin  10
// meaningful names for the serial ports
#define Monitor Serial2
#define ASCOM   Serial
#define Encoder Serial1

//liquid crystal two lines below
//const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//CREATE INSTANCE OF STEPPER MOTOR

AccelStepper  stepper(AccelStepper::DRIVER, stepPin, dirPin, true);

String  receivedData;
float   TargetAzimuth, CurrentAzimuth;
boolean DoTheDeceleration;
boolean SlewStatus;             // controls whether the stepper is stepped in the main loop
float   StepsPerSecond;         // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25

boolean TargetChanged = false;

float   normalAcceleration;                            // was incorrectly set to data type int

int     DecelValue                  = 800;                // set after empirical test Oct 2020
int     EncoderReplyCounter         = 0;

long    pkstart                     = 0.0l;              // note i after 0.0 denotes long number - same type as millis()


String  lcdblankline = "                    ";  //twenty spaces to blank lcd display lines
String  TargetMessage = lcdblankline;
String  QueryDir ="No Direction";
String  movementstate;
String  pkversion = "5.0";
//TODO REMOVE THE LINE BELOW - TESTING ONLY
//String globalreceipt= "3";
/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/


void setup()
{

  //  changed the following line for the 4809 context 
  pinMode(9, INPUT_PULLUP);                   // see the notes in github. this pulls up the serial Rx pin to 5v.


  // It transformed the workings of the serial link to the encoder
  stepper.stop();                               // set initial state as stopped
  // Change below to suit the stepper

  SlewStatus = false;
  StepsPerSecond = 300.0;                       // changed following empirical testing Oct 2020
  normalAcceleration = 140.0;                    // changed following empirical testing October 17th 2020 - changed from 40 to 20 for trial
  stepper.setMaxSpeed(StepsPerSecond);          // steps per second see below -
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(normalAcceleration);  // steps per second per second.
  // Note V= acceleration * time, so a vlue of e.g. 1 step /s/s takes 10 secs to reach maxspeed of 10 or 15 secs to reach maxspeed 15 etc
  //
  // see how the speed of 15 goes empirically
  // with a 10 cm diameter drive wheel on the shaft, calculations show 20 shaft rotations
  // are required to move the dome through approx one revolution and this works out to be 27600 steps for one dome rotation


  // initialise

  CurrentAzimuth    = 0.0;
  DoTheDeceleration = true;      // used to set deceleration towards target azimuth
  pkstart           = millis();

  //lcd.begin(20, 4);                      // 20 columns x 4 rows
  //lcd.clear();
  //lcdprint(0, 0, "MCU-stepper Ready");
  //lcdprint(0, 1, "Version " +  pkversion);

  //Serial.println("The target is about to be initialised ");




  // Serial.print("The value of getcurrentazimuth function is  ");
  // Serial.println (getCurrentAzimuth(CurrentAzimuth));

  //Serial.print("The target has been initialised to ");
  // Serial.println (TargetAzimuth);
  delay(2000);

  ASCOM.begin(19200) ;                        // start serial ports ASCOM driver - usb with PC - rx0 tx0 and updi
  Encoder.begin(19200);                        // Link with the Encoder MCU
  Monitor.begin(19200);                        // serial with the Monitor program

  //todo remove 3 lines below
 // delay(25000);   //this gives time to get to the serial port - as the 4809 does not reset on sermon start
//ASCOM.print(globalreceipt);
//ASCOM.println(" ");
  TargetAzimuth =  getCurrentAzimuth();        // 
  //todo remove 2 lines below
//ASCOM.print(globalreceipt);
//ASCOM.println(" ");
  // Serial.println(F_CPU);    // print the cpu speed


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


  if (ASCOM.available() > 0)                              // when serial data arrives from the driver on USB capture it into a string
  {

    receivedData = ASCOM.readStringUntil('#');          // read a string from PC serial port usb

    /*
        if (receivedData.startsWith("TEST", 0))
        {
          lcd.setCursor(0, 0);
          lcd.print("Test Received");
          delay(1000);
          lcd.clear();
          Serial.println("Communications established, received  " + receivedData);
          Serial.println("Target Az  is " +  String( TargetAzimuth));
          Serial.println("Current Az is " + String( CurrentAzimuth));

          Serial.println(" so a good sequence would be as follows IN THE ORDER RECEIVED FROM the driver....");
          Serial.println("1 SL#      to check slewing");
          Serial.println("2 SA220#   slew to azimuth request");
          Serial.println("3 SL#      to check slewing");
          Serial.println("4 SL#      to check slewing");
          Serial.println("5 SL#      to check slewing etc etc");
          Serial.println(" ");
          Serial.println("emergency stop is ES#");


          //
          receivedData = "";
        }
    */



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

      TargetAzimuth = receivedData.toFloat();    // store the target az for comparison with current position
      TargetChanged = true;

      //  Serial.println();
      //  Serial.print("in slewto target received ");
      //  Serial.println(TargetAzimuth);

      if (SlewStatus == false)             // only do this if not slewing
      {
        SlewStatus = true;
        stepper.setAcceleration(normalAcceleration);      // set the acceleration
        stepper.setCurrentPosition(0);                    // initialise the stepper position
        QueryDir = WhichDirection();                         // work out which direction of travle is optimum

        if (QueryDir == "clockwise")
        {
          // Serial.println("setting stepper target position to 100000");
          stepper.moveTo(100000);                         // positive number means clockwise in accelstepper library

        }

        if (QueryDir == "anticlockwise")
        {
          //  Serial.println("setting stepper target position to -100000");
          stepper.moveTo(-100000);                      // negative is anticlockwise in accelstepper library

        }
        // Serial.print("in slewto azimuth querydir is ");
        // Serial.print(QueryDir);

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
      //send true or false

      if (SlewStatus)
      {
        ASCOM.print("Moving#" );
        //todo remove the line below
      //ASCOM.println(globalreceipt);
        // Serial.println("#");

      }
      else
      {
        ASCOM.print("Notmoving#");             // sent to serial USB and picked up by the driver
        //todo remove the line below
       // ASCOM.println(globalreceipt);
        // Serial.println("#");

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
    if (  (millis() - pkstart) > 1000.0  )                // half second checks for azimuth value as the dome moves
    {

      UpdateThelcdPanel();


      // CHECK HERE IF Monitor Serial is available - request from monitor program

      pkstart = millis();

    }

  }


  if (    abs( stepper.distanceToGo() ) < 20   )
  {
    SlewStatus = false;                      // used to stop the motor in main loop
    movementstate  = "Stopped.  ";           // for updating the lcdpanel

    // Serial.print("ABS STEPPER distance to go....");
    // Serial.println();
    //update the LCD
    TargetMessage = "Target achieved ";
    UpdateThelcdPanel();
    // lcdprint(0, 3, "Target achieved     "); // update the LCD with the good news
    //  lcdprint(0, 2, lcdblankline);

    PowerOff();                                // power off the stepper

  }
  else
  {
    movementstate  = "Moving";              // for updating the lcdpanel
    TargetMessage = "Awaiting Target ";
    stepper.run();
  }



} // end void Loop //////////////////////////////////////////////////////////////////////////////////////////////////////


void Emergency_Stop(float azimuth, String mess)
{

  stepper.stop();
  SlewStatus = false;


 // lcdprint(0, 0, lcdblankline);
 // lcdprint(0, 0, "Stopped");
 // lcdprint(0, 1, mess);

  // todo turn off power to the stepper
  PowerOff();
}

void lcdprint(int col, int row, String mess)
{
  // lcd.clear();
 // lcd.setCursor(col, row);
  // lcd.print(mess);

}

String WhichDirection()
{ // this routine decides which direction to go based on the difference betwen current and target azimuth
  int DiffMod;
  String dir ;

  DiffMod = AngleMod360();

  if (DiffMod >= 180)
  {
    dir = "clockwise";      // the increasing Azimuth case
  }
  else
  {
    dir = "anticlockwise";  // the decreasing Azimuth case
  }

  return dir;
  // code above optimises movement to take the shortest distance
  // Serial.print("which direct ? dir is ");
  // Serial.println(dir);

  //delay (1000);   // remove for test only
}

void WithinFiveDegrees()
{

  if (DoTheDeceleration)
  {

    CurrentAzimuth =   getCurrentAzimuth();

    if (     (abs(CurrentAzimuth - TargetAzimuth) < 5)    && (TargetChanged == true)   )                       // within 5 degrees of target
    {
      //Serial.println("Test of how many calls to LESS than 5 degrees ");
      DoTheDeceleration = false;
      if (QueryDir == "clockwise")
      {
        // set the moveto position to allow 100 steps more for deceleration  +ve for clockwise -ve for anticclock
        //    stepper.setCurrentPosition(0);
        stepper.moveTo(stepper.currentPosition() + DecelValue); //FROM MA860H Datasheet @0.225 step angle, it requires 1600 steps per rotation
        //of the stepper drive wheel, so 1000 is 0.6 of a rotation
        //  Serial.println();
        //  Serial.print("stepper clockwise decel pos is ");
        //  Serial.println(stepper.currentPosition() + 50);
        //  Serial.println();
      }

      if (QueryDir == "anticlockwise")
      {
        //  stepper.setCurrentPosition(0);
        stepper.moveTo(stepper.currentPosition() - DecelValue);             // check this by printing out current position is it negative?
        //  Serial.println();
        //  Serial.print("stepper anticlockwise decel pos is ");
        //  Serial.println(stepper.currentPosition() - 50);
        //  Serial.println();
      }
    }

  }


}

float getCurrentAzimuth()
{
  float az;

  // test serial - not serial3 - remove this after testing
  // Serial.println();
  // Serial.print ( "Sent AZ# to the encoder and got back ");    // remove after test


  //
  boolean validaz = false;

  while (validaz == false)
  {

    Encoder.print("AZ#");          //this is sent to the encoder which is coded to return the azimuth of the dome
//todo evaluate new line below
    delay(100);  //Some testing showed this line improves reliability of the send /receive cycle
    if (Encoder.available() > 0)                       // when serial data arrives capture it into a string
    {

      String receipt = Encoder.readStringUntil('#');   // read a string from the encoder
      //TODO REMOVE THE LINE BELOW WHICH WAS FOR TESTING
      //globalreceipt= receipt;
      az = receipt.toFloat();                          // convert

      if (  (az > 0) && (az <= 360) )
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

void UpdateThelcdPanel()
{

SendToMonitor();

  /*
  if (Serial1.available() > 0)        // serial 1 is the monitor program link
  {
  String MonitorRequest ="";
  MonitorRequest=  Serial1.readStringUntil('#');
  if (MonitorRequest.indexOf("Ping", 0) > -1)     
  {
   // Serial.print("sending");
    SendToMonitor();
   // Serial.println("sent");
  }
  
  }
  */
  // this sends the data to the monitor program

  // for the new Arduino Monitor Winforms app, include 'EncoderReplyCounter' in the update


  //stepper.run();
  // update lcd panel
  //lcdprint(0,  0, lcdblankline);
  //lcdprint(0,  1, lcdblankline);
  //lcdprint(0,  2, lcdblankline);
  //lcdprint(0,  3, lcdblankline);

  
  //lcdprint(0,  0, "Goto request        ");
  //stepper.run();
  //lcdprint(15, 0, String(int(TargetAzimuth)));
  //stepper.run();

  //lcdprint(0,  1, "Status:  " + movementstate);
  //stepper.run();

  //lcdprint(7,  2, QueryDir);
  //stepper.run();

  //lcdprint(0, 3, TargetMessage);
  //stepper.run();
  //lcdprint(16, 3, "   ");
  /*if (QueryDir =="clockwise")
    {
      lcdprint(16, 3,  String(360 - AngleMod360() ) );    // try this to check if the distance to go is correct
    }
    else
    {

      lcdprint(16, 3,  String(AngleMod360() ) );    // try this to check if the distance to go is correct 
    }
  

  stepper.run();
*/
}
int AngleMod360()
{
  // This routine looks at the current azimuth and the target azimuth and returns a value which is the difference modulo 360
  // the reason this is coded is that it gives a different result from C++'s MOD function.
  // See google sheets for the derivation algoritm - the sheet is called Target Azimuth Scenarios.

  int difference = 0;
  int part1 = 0;
  int part2 = 0;
  int Modresult = 0;


  CurrentAzimuth = getCurrentAzimuth();

  difference = (int)(CurrentAzimuth - TargetAzimuth );
  part1 = (int)(difference / 360);
  if (difference < 0)
  {
    part1 = -1;
  }
  part2 = part1 * 360;
  Modresult  = difference - part2;

  return Modresult;
}

void SendToMonitor()
{

 // Serial.println("START#");
 // Serial.println(String(int(TargetAzimuth))   + '#');
 // Serial.println(movementstate                + '#');
 // Serial.println(QueryDir                     + '#');
 // Serial.println(TargetMessage                + '#');
    
 // Serial.println(String(EncoderReplyCounter)  + '#');


  Monitor.print("START#");
  Monitor.print(String(int(TargetAzimuth))   + '#');
  Monitor.print(movementstate                + '#');
  Monitor.print(QueryDir                     + '#');
  Monitor.print(TargetMessage                + '#');
  if (QueryDir =="clockwise")            // see google sheets for the distance to target formula -Target Azimuth Scenarios
  {

    // try testing the if stmt below to stop the Monitor program showing distance to go = 360 if there's a slight overun of the dome
    // note the todo at the end of the if clause if bringing the if - else statement into use
/*
    if (  (360-AngleMod360()) < 1 )
    {
      Monitor.print("0");
    }
    else
    {
    Monitor.print(String(360 - AngleMod360() )       + '#'); 
    }
  */
  //todo remove the line below if using the if statement above
  Monitor.print(String(360 - AngleMod360() )       + '#'); 
  
    //Serial.println(String(AngleMod360())        + '#');        // note this is a test print from the block above and is serial not serial1
  }
  else   //querydir is anticlockwise
  {
  Monitor.print(String(AngleMod360() )       + '#');         
  }
  Monitor.print(String(EncoderReplyCounter)  + '#');
  /*
  list of data need by the monitor program
  targetazimuth
  movementstate
  querydir
  targetmessage
  anglemod360
  encoderreplycounter
  */
}
void PowerOn()                          // set the power SSR gate high
{
digitalWrite(power_pin,      HIGH);

delay(2000);                            // gives time for the MA860H unit to power on and stabilise
}

void PowerOff()                         // set the power SSR gate low
{
digitalWrite(power_pin,      LOW);
}
