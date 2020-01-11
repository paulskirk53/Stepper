//verion A1.4 - change the variable in setup too
// check the arrivedatdestination moveto values as they may need empirical change on testing
// This routine accepts these commands from the ASCOM Driver via USB Serial Cable:
//TEST#
//ES# - emergency stop
//SA999.99# - Slew to azimuth
//SL# - Slew status request
//
// The routine drives the stepper motor to move the Dome
// It acquires the current azimuth via hardware serial3 from the encoder

// library for stepper

#include <AccelStepper.h>
#include <LiquidCrystal.h>

// Define a stepper and the pins it will use

// pin definitions for step, dir and enable

#define                stepPin 7             // step pin tested and works - motor moves
#define                dirPin  8
#define                enaPin  9             // presently n/c - the enable pin

//liquid crystal two lines below
const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//CREATE INSTANCE OF STEPPER MOTOR

AccelStepper  stepper(AccelStepper::DRIVER, stepPin, dirPin, true);

String receivedData;
double TargetAzimuth, CurrentAzimuth;

boolean do_once;
boolean SlewStatus;           // controls whether the stepper is stepped in the main loop
float StepsPerSecond;         // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25

boolean TargetChanged = false;

int  normalAcceleration;
int  lower_limit   = 0;
int  upper_limit   = 360;
long pkinterval    = 0;
long pkstart       = 0;
long PKcurrentTime = 0;

String lcdblankline = "                    ";  //twenty spaces to blank lcd display lines
String QueryDir;
String movementstate;
String pkversion = "A1.4";
/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/


void setup()
{
  // put your setup code here, to run once:

  Serial.begin(19200) ;                        // start serial ports - usb with PC
  Serial3.begin(19200);                        // start usb with encoder
  stepper.stop();                               // set initial state as stopped
  // Change below to suit the stepper

  SlewStatus = false;
  StepsPerSecond = 500.0;                       // changed following empirical testing
  normalAcceleration = 50;                       // changed following empirical testing
  stepper.setMaxSpeed(StepsPerSecond);          // steps per second see below -

  stepper.setAcceleration(normalAcceleration);  // steps per second per second.
  // Note V= acceleration * time, so a vlue of e.g. 1 step /s/s takes 10 secs to reach maxspeed of 10 or 15 secs to reach maxspeed 15 etc
  //
  // see how the speed of 15 goes empirically
  // with a 10 cm diameter drive wheel on the shaft, calculations show 20 shaft rotations
  // are required to move the dome through approx one revolution and this works out to be 27600 steps for one dome rotation


  // initialise

  CurrentAzimuth = 0.0;
  do_once = true;      // used to set deceleration towards target azimuth
  pkstart = millis();

  lcd.begin(20, 4);                      // 20 columns x 4 rows
  lcd.clear();
  lcdprint(0, 0, "MCU-stepper Ready");
  lcdprint(0, 1, "Version " +  pkversion);


  TargetAzimuth =  getCurrentAzimuth();
  // Serial.print("The value of getcurrentazimuth function is  ");
  // Serial.println (getCurrentAzimuth(CurrentAzimuth));

  Serial.print("The target has been initialised to ");
  Serial.println (TargetAzimuth);
  delay(5000);
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

  if (Serial.available() > 0)                              // when serial data arrives from the driver on USB capture it into a string
  {

    receivedData = Serial.readStringUntil('#');          // read a string from PC serial port usb


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


    if (receivedData.startsWith("ES", 0))               // Emergency stop requested from C# driver
    {
      lcd.clear();
      Emergency_Stop(0.00, "Received ES");
      receivedData = "";
    }                                                   // end Emergency Stop else clause



    //*************************************************************************
    //******** code for SA process below **************************************
    //**** example of data sent by driver SA220.00#  **************************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("SA", 0) > -1) //
    {
      // strip off 1st 2 chars
      receivedData.remove(0, 2);

      TargetAzimuth = receivedData.toFloat();    // store the target az for comparison with current position
      TargetChanged = true;
      Serial.println();
      Serial.print("in slewto target received ");
      Serial.println(TargetAzimuth);

      if (SlewStatus == false)             // only do this if not slewing
      {

        stepper.setAcceleration(normalAcceleration);      // set the acceleration

        QueryDir = WhichDirection();                         // work out which direction of travle is optimum

        if (QueryDir == "clockwise")
        {

          stepper.moveTo(100000);                         // positive number means clockwise in accelstepper library
          SlewStatus = true;
        }

        if (QueryDir == "anticlockwise")
        {
          stepper.moveTo(-100000);                      // negative is anticlockwise in accelstepper library
          SlewStatus = true;
        }
        Serial.print("in slewto azimuth querydir is ");
        Serial.print(QueryDir);


        //  if ((TargetAzimuth < lower_limit ) || (TargetAzimuth > upper_limit))   //error trap azimuth value
        //  {
        //    Emergency_Stop(TargetAzimuth, "Target Az failure   ");
        //  }


        receivedData = "";

      }

    }

    //*************************************************************************
    // ******** code for SL process below *************************************
    //**** example of data sent by driver SL220.00#  **************************
    //*************************************************************************
    //

    if (receivedData.indexOf("SL", 0) > -1) //

    {
      //send true or false

      if (SlewStatus)
      {
        Serial.print("Moving");
        Serial.println("#");

      }
      else
      {
        Serial.print("Notmoving");               // sent to serial USB and picked up by the driver
        Serial.println("#");

      }
      receivedData = "";

    }  // end SL case


  }  // end software serial

  //CHECK ONCE PER two SECONDs FOR CURRENT AZIMUTH

  PKcurrentTime = millis();

  pkinterval = PKcurrentTime - pkstart ;


  if (pkinterval > 2000 )
  {
    ArrivedAtDestinationCheck();
    pkinterval = 0;
    pkstart = millis();
    //Serial.println("should be 2 secs  ");   //test only

  }

  if (SlewStatus)                    // if the slew status is true, run the stepper and check for decel and stopping
  {

    stepper.run();

  }


} // end void Loop


void Emergency_Stop(double azimuth, String mess)
{

  stepper.stop();
  SlewStatus = false;


  lcdprint(0, 0, lcdblankline);
  lcdprint(0, 0, "Stopped");
  lcdprint(0, 1, mess);


}

void lcdprint(int col, int row, String mess)
{
  //lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(mess);

}

String WhichDirection()
{
  // put in the code from the driver which does modulo stuff
  long difference = 0;
  long part1 = 0;
  long part2 = 0;
  long part3 = 0;
  long DiffMod = 0;
  String dir ;

  CurrentAzimuth = getCurrentAzimuth();

  difference = (int)(CurrentAzimuth - TargetAzimuth );
  part1 = (int)(difference / 360);
  if (difference < 0)
  {
    part1 = -1;
  }
  part2 = part1 * 360;
  part3 = difference - part2;
  DiffMod = part3;

  if (DiffMod >= 180)
  {
    dir = "clockwise"; // the clockwise case
  }
  else
  {
    dir = "anticlockwise";  //counerclockwise}
  }

  return dir;
  // code above optimises movement to take the shortest distance
  Serial.print("which direct ? dir is ");
  Serial.println(dir);

  delay (1000);   // remove for test only
}

void ArrivedAtDestinationCheck()
{

  if (do_once)
  {
    //Serial.print("do once ");
    delay (1000);  // remove
    CurrentAzimuth =   getCurrentAzimuth();

    if (     (abs(CurrentAzimuth - TargetAzimuth) < 5)    && (TargetChanged == true)   )                    // within 5 degrees of target
    {
      do_once = false;
      if (QueryDir == "clockwise")
      {
        // set the moveto position to allow 100 steps more for deceleration  +ve for clockwise -ve for anticclock

        stepper.moveTo(stepper.currentPosition() + 150);

      }

      if (QueryDir == "anticlockwise")
      {

        stepper.moveTo(stepper.currentPosition() - 150);             // check this by printing out current position is it negative?

      }
    }

  }

  if (    abs( stepper.distanceToGo() < 20 )    )
  {
    SlewStatus = false;                      // used to stop the motor in main loop
    movementstate  = "Not Moving";           // for updating the lcdpanel

    Serial.print("Stopped now....");
    Serial.println();
  }
  else
  {
    movementstate  = "Moving";              // for updating the lcdpanel
  }

}

double getCurrentAzimuth()
{
  double az;

  // test serial - not serial3 - remove this after testing
  // Serial.println();
  // Serial.print ( "Sent AZ# to the encoder and got back ");    // remove after test


  //
  boolean validaz = false;

  while (validaz == false)

  {
    Serial3.print("AZ#");          //this is sent to the encoder which is coded to return the azimuth of the dome
    delay(200);  // for response to arrive

    if (Serial3.available() > 0)                            // when serial data arrives capture it into a string
    {

      String receipt = Serial3.readStringUntil('#');        // read a string from the encoder
      az = receipt.toDouble();                              // convert
      if (  (az > 0) && (az <= 360) )
      {

        // Serial.print("in az validation az is ");
        //  Serial.println(receipt);                             //remove this after testing as it destroye the protocol integrity

        validaz = true;

      }
    }
    //convert receipt to double as az and return it

  }
  return az;
}

void UpdateThelcdPanel()
{

  // update lcd panel
  lcdprint(0,  0, lcdblankline);
  lcdprint(0,  1, lcdblankline);
  lcdprint(0,  2, lcdblankline);
  lcdprint(0,  3, lcdblankline);


  lcdprint(0,  0, "Goto request");
  lcdprint(15, 0, String(int(TargetAzimuth)));


  lcdprint(0,  1, "Status:  " + movementstate);
  lcdprint(7,  2, QueryDir);

  lcdprint(0, 3, "Distance to go");

  lcdprint(16, 3,  String(int(TargetAzimuth -  getCurrentAzimuth())));
  if (SlewStatus == false)
  {
    lcdprint(0, 3, "Target achieved     "); // update the LCD with the good news
  }


}
