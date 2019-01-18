// set slew status as false in setup. don't cal 20 deg, 5 deg or distancechecker unless slewstatus is true
// if targetaz < min or targetaz > max need to do something to avoid collision at pulley//
//
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

boolean endpointdone;
boolean SlewStatus;           // controls whether the stepper is stepped in the main loop
float StepsPerSecond;         // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25
boolean Clockwise;
boolean DecelFlag;
int normalAcceleration;

/*
--------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------
*/


void setup()
{
	// put your setup code here, to run once:

	Serial.begin(9600);                    // start serial ports - usb with PC
	stepper.stop();                         // set initial state as stopped
	// Change below to suit the stepper
	DecelFlag = false;
	SlewStatus = false;
	StepsPerSecond = 30.0; // changed from 15 to 30 on 3-10-18 as a speed up test
	normalAcceleration = 1;
	stepper.setMaxSpeed(StepsPerSecond);			   // steps per second see below -
	// the controller electronics is set to 0.25 degree steps, so 15 stepspersecond*0.25= 3.75 degrees of shaft movement per second
	stepper.setAcceleration(normalAcceleration);     // steps per second per second.
	// Note V= acceleration * time, so a vlue of e.g. 1 step /s/s takes 10 secs to reach maxspeed of 10 or 15 secs to reach maxspeed 15 etc
	//
	// see how the speed of 15 goes empirically
	// with a 10 cm diameter drive wheel on the shaft, calculations show 20 shaft rotations
	// are required to move the dome through approx one revolution and this works out to be 27600 steps for one dome rotation



	// initialise slewtoAz, currentazimuth
	TargetAzimuth = 270.0;                 //these two need to be separated by at least 20 degrees as the initial starting condition
	CurrentAzimuth = 180.0;

	

	/*
	default pos for MA860H controller switches:
	5     6    7    8
	on   off  off  off = smallest step size

	my setting:
	5     6    7    8
	on   on  off  on =  step angle ~ 0.25 degree

	*/

	lcd.begin(20, 4);              // 20 columns x 4 rows
	

	endpointdone = false;   // to facilitate one time execution of the endpoint setting when within 5 degrees of target


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

	if (Serial.available() > 0)                            // when serial data arrives capture it into a string
	{

		receivedData = Serial.readStringUntil('#');          // read a string from PC serial port usb

	//	Serial.println(" 1 received  " + receivedData);    //TEST ONLY REMOVE

		if (receivedData.startsWith("TEST", 0))
		{
			lcd.setCursor(0, 0);
			lcd.print("Test Received");
			delay(1000);
			lcd.clear();
			delay(1000);
			lcd.print("Test again");
			Serial.println("Communications established, received  " + receivedData);
			Serial.println("Target Az   " +  String( TargetAzimuth));
			Serial.println("Current Az   " + String( CurrentAzimuth));

			Serial.println(" so a good sequence would be as follows IN THE ORDER RECEIVED FROM the driver....");
			Serial.println("1 CL#      clockwise movement request");
			Serial.println("2 SA220#   slew to azimuth request");
			Serial.println("3 SL180#   simulates the compass routine providing 180 degrees");
			Serial.println("4 SL201#   tests the within 20 degrees bit which should reduce speed to one third");
			Serial.println("5 SL218#   should stop when this is entered because we are within the coded 5 degree window which defines target reached.");
			Serial.println(" ");
			Serial.println("emergency stop is ES#");

			// emergency stop is ES#
			//

		}


		if (receivedData.startsWith("ES", 0))               // Emergency stop requested from C# driver
		{

			Emergency_Stop(TargetAzimuth, "Received ES");
		}                                                   // end Emergency Stop else clause



		//*************************************************************************
		//******** code the SA process below **************************************
		//**** example of data sent by driver SA220.00#  **************************
		//**** SA command is followed in the driver by sending CL# or CC# *********
		//*************************************************************************

		if (receivedData.startsWith("SA", 0)) // SlewToAzimuth command from C# driver
		{
			
			
			// strip off 1st 2 chars
			receivedData.remove(0, 2);


			TargetAzimuth = receivedData.toFloat();    // store the target az for comparison with current position
			// if target < min or target > max need to do something to avoid collision at pulley

			lcd.setCursor(0, 0);
		    lcd.print("Target Az: ");
			lcd.setCursor(13,0);
			lcd.print(receivedData);


			if ((TargetAzimuth < 10.0 ) || (TargetAzimuth >350.0))   //error trap azimuth value
			{
			Emergency_Stop(TargetAzimuth, "Target Az failure   ");
			}
			

			receivedData = "";
				
		}



		if (receivedData.startsWith("CL", 0))        //   clockwise Slew command from C# driver
		{

			// used 30000 as one full rev of dome and this should therefore cover any size slew

			stepper.setMaxSpeed(StepsPerSecond);           //  must call this following moveto
			stepper.setAcceleration(normalAcceleration);
			SlewStatus = true;
			Clockwise = true;                           // used for deceleration
			DecelFlag = false;                  // need to set this here
			
			stepper.setCurrentPosition(15)  ;                //outside the aceel/ decel range checker
			stepper.moveTo(30000);                      //  Negative is anticlockwise pos is clockwise from the 0 position.
			stepper.run();
			lcd.clear();
			lcd.setCursor(0, 2);
			lcd.print("Moving Clockwise");
			
			receivedData = "";


		} // end if cl

		if (receivedData.startsWith("CC", 0)) //  counter clockwise Slew command from C# driver
		{

			stepper.setMaxSpeed(StepsPerSecond);           //  must call this following moveto
			stepper.setAcceleration(normalAcceleration);
			SlewStatus = true;
			Clockwise = false;                           // used for deceleration
			DecelFlag = false;                           // need to set this here
			stepper.setCurrentPosition(-15)    ;         // outside the aceel/ decel range checker
			stepper.moveTo(-30000);                      // Negative is anticlockwise pos is clockwise from the 0 position.
			stepper.run();
			lcd.clear();
			lcd.setCursor(0, 2);
			lcd.print("Moving Anticlockwise");
			
			receivedData = "";
			//write the direction to the LCD screen

		} // end if cc



		//*************************************************************************
		// ******** code the SL process below *************************************
		//**** example of data sent by driver SL220.00#  **************************
		//*************************************************************************
		//

		if (receivedData.startsWith("SL", 0)) // Receive the Current AZ via driver from Compass
		{

			// strip off 1st 2 chars
			receivedData.remove(0, 2);


			CurrentAzimuth = receivedData.toFloat();    // store the target az for comparison with current position

						//PRINT TO LCD
			lcd.setCursor(0, 1);
			lcd.print("Current Az: ");
			lcd.setCursor(13,1);
			lcd.print(receivedData);

			if ((CurrentAzimuth < 10.0 ) || (CurrentAzimuth >350.0))   //error trap azimuth value
			{
				Emergency_Stop(CurrentAzimuth, "Current Az failure  ");
			}



			
			receivedData = "";
			
			if (SlewStatus)
			{
				Serial.print("Moving");
				Serial.println("#");
			}
			else
			{
				Serial.print("Notmoving");               // sent to serial USB and picked up by the driver
				Serial.println("#");
				endpointdone=false;                      //reset the flag which controls one time execution of the 5 degree window
			}
			
		}  // end SL case


	}  // end software serial

	if (SlewStatus)                    // if the slew status is true, run the stepper and check for decel and stopping
	{

		within_twenty_degrees();             //check at start - what value is 212 going to cause
		within_five_degrees();               //check at start - what value is 212 going to cause
		distancechecker();                   // Check how close to the endpoint and reset flags for motor stop and initialisation of variables


		stepper.run();
	}
	
	

} // end void

void distancechecker()
{


	
	if (abs(stepper.distanceToGo()) < 10)               // within 10 steps of target
	{
		SlewStatus = false;                             //used to stop the motor in main loop
		DecelFlag = false;                              // reset this so that decel can happen again when new move commands come in
		endpointdone = false;                           // RESET this so that the 5 degree window for stopping is enabled
		stepper.setAcceleration(normalAcceleration);
		TargetAzimuth= CurrentAzimuth +25.0;
		lcd.setCursor(0, 2);
		lcd.print("Movement Stopped.   ");
		
	}

}



void within_five_degrees()
{

	// now code cases 1 and 2 from arduino stepper process in spreadsheet
	// which compare currentazimuth with targetazimuth

	// 1 current az and target az are within 5 degrees
	//problem below is repeated sl requests don't allow the motor to stop.

	if ((abs(CurrentAzimuth - TargetAzimuth) < 5.0  ) && (endpointdone == false))
	{

		endpointdone=true;

		// new code for v4
		if (Clockwise)
		{
		  stepper.moveTo(stepper.currentPosition() + 100);             // set the end point so deceleration can happen
		}
		else                                    // else clause is counterclockwise movement of stepper
		{
			stepper.moveTo(stepper.currentPosition() - 100);
		}

	}  // end true case

	// if the two angles are not within 5 deg then the motor is still going


}

void within_twenty_degrees()
{

	// decelerate if within 20 degrees to prevent overshoot
	//


	if ((abs(CurrentAzimuth - TargetAzimuth) < 20.0) && (DecelFlag == false))  // within 20 degrees of target....
	{
		
		DecelFlag = true;                             // set the flag so this code is only executed once
		stepper.setMaxSpeed(StepsPerSecond * 0.25);   // reduce speed to one HALF - clockwise dir
		stepper.setAcceleration(normalAcceleration * 2);
		stepper.run();
		
	}

}
void Emergency_Stop(double azimuth, String mess)
{

stepper.stop();
SlewStatus = false;
DecelFlag = false;
endpointdone = false;

lcd.clear();
lcd.setCursor(0, 0);
lcd.print("Range error -Stopped");

lcd.setCursor(0, 1);
lcd.print(mess);

lcd.setCursor(0, 2);
lcd.print(azimuth);
}
