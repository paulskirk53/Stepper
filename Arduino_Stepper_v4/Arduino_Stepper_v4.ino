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
float TargetAzimuth, CurrentAzimuth;

boolean do_once;
boolean SlewStatus;           // controls whether the stepper is stepped in the main loop
float StepsPerSecond;         // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25

boolean Clockwise;

int normalAcceleration;
int lower_limit = 0;
int upper_limit = 360;
long pkinterval = 0;
long pkstart    = 0;
long pkfinish   = 0;

String lcdblankline = "                    ";  //twenty spaces to blank lcd display lines
String QueryDir;

/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/


void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200) ;                        // start serial ports - usb with PC
  Serial3.begin(115200);                        // start usb with encoder
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
  TargetAzimuth =  0.0;
  CurrentAzimuth = 0.0;
  do_once = true;      // used to set deceleration towards target azimuth


  lcd.begin(20, 4);                      // 20 columns x 4 rows
  lcd.clear();
  lcdprint(0, 0, "MCU-stepper Ready");



} // end setup

/*
  \\\\\\\\\\\\\\\\/////////////////////////\\\\\\\\\\\\\\\\\\\\\///////////////
  ////////////////\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\
  ////////////////\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\
  \\\\\\\\\\\\\\\\/////////////////////////\\\\\\\\\\\\\\\\\\\\\///////////////
*/


void loop()
{
  pkstart = millis();
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

      // emergency stop is ES#
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
      // if target < min or target > max need to do something to avoid collision at pulley


      if (SlewStatus == false)             // only do this if not slewing
      {

        stepper.setAcceleration(normalAcceleration);      // set the acceleration

        WhichDirection(QueryDir);                         // work out which direction of travle is optimum

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

        lcdprint(0,  0, lcdblankline);
        lcdprint(0,  0, "Goto requested");
        lcdprint(15, 0, receivedData);
        lcdprint(0,  2, lcdblankline);
        lcdprint(0,  2, String(SlewStatus));

        lcdprint(0, 3, lcdblankline);
        lcdprint(0, 3,  QueryDir);

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

  ArrivedAtDestinationCheck();

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

void WhichDirection(String dir)
{
  // put in the code from the driver which does modulo stuff
  long difference = 0;
  long part1 = 0;
  long part2 = 0;
  long part3 = 0;
  long DiffMod = 0;


  getCurrentAzimuth(CurrentAzimuth);

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


  // code above optimises movement to take the shortest distance

}

void ArrivedAtDestinationCheck()
{

  if (do_once)
  {

    getCurrentAzimuth(CurrentAzimuth);

    if (abs(CurrentAzimuth - TargetAzimuth) < 5)                     // within 5 degrees of target
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

  if (stepper.distanceToGo() < 20)
  {
    SlewStatus = false;                             // used to stop the motor in main loop
  }



  lcdprint(0, 2, lcdblankline);
  lcdprint(0, 2, "Movement Stopped.   ");
  lcdprint(0, 4, lcdblankline);
  lcdprint(0, 4, "Target achieved.    ");


}

void getCurrentAzimuth(double az)
{
  long interval = 0;
  pkstart = millis();           //use this eventually to control a timeout for serial tx / rx

  //test serial - not serial3 - remove this after testing

  Serial.print ( "Sent AZ# to the encoder and got back ");    // remove after test


  //

  Serial3.print("AZ#");          //this is sent to the encoder which is coded to return the azimuth of the dome

  while (  !(Serial3.available() > 0)  )
  {
    //retry after a 2 second wait
    interval = millis() - pkstart;
    if (interval > 2000)
    {
      Serial3.print("AZ#");                                  //resend if there was no response within 2 seconds
      pkstart = millis();
    }
  }

  
  if (Serial3.available() > 0)                            // when serial data arrives capture it into a string
  {

    String receipt = Serial3.readStringUntil('#');        // read a string from PC serial port usb the # is not included by arduino
    az = receipt.toDouble();                              // convert
    
    Serial.println(receipt);                             //remove this after testing as it destroye the protocol integrity
  }

  //convert receipt to double as az and return it



}
