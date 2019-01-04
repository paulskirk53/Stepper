
//
// THE serial prints numbers 2 to 4 in SA fail - try a different board
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
	// Note V= acceleration * time, so a vlue of e.g. 1 step /s/s takes 10 secs to reach maxspeed of 10
	// so with the values we have set currently it takes 15 seconds
	// to get to the speed of 15 see how this goes empirically
	// with a 10 cm diameter drive wheel on the shaft, calculations show 20 shaft rotations
	// are required to move the dome through approx one revolution and this works out to be 27600 steps for one dome rotation



	stepper.setCurrentPosition(1);      // new in v4



	// initialise slewtoAz, currentazimuth
	TargetAzimuth = 212.0;                 //these two changed from zero for test
	CurrentAzimuth = 212.0;


	/*
	default pos for electronics:
	5     6    7    8
	on   off  off  off = smallest step size

	my setting:
	5     6    7    8
	on   on  off  on =  step angle ~ 0.25 degree

	*/

	//lcd.begin(20, 4);              // 20 columns x 4 rows

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

	/*

	// set the cursor to column 0, line 0
	// (note: line 1 is the second row, since counting begins with 0):
	lcd.setCursor(0, 0);
	lcd.print("Target Azimuth : ");
	lcd.setCursor(0, 1);
	lcd.print(lcdazimuth);

	*/




	if (Serial.available() > 0)                            // when serial data arrives capture it into a string
	{

		receivedData = Serial.readStringUntil('#');          // read a string from PC serial port usb

		Serial.println(" 1 received  " + receivedData);    //TEST ONLY REMOVE

		if (receivedData.startsWith("TEST", 0))
		{
			Serial.println("Communications established, received  " + receivedData);
			Serial.println("Target Az   " +  String( TargetAzimuth));
			Serial.println("Current Az   " + String( CurrentAzimuth));

			Serial.println(" so a good sequence would be as follows");
			Serial.println("1 SA220#   slew to azimuth request");
			Serial.println("2 CL#      clockwise movement request");
			Serial.println("3 SL180#   simulates the compass routine providing 180 degrees");
			Serial.println("4 SL201#   tests the within 20 degrees bit which should reduce speed to one third");
			Serial.println("5 SL218#   should stop when this is entered because we are within the coded 5 degree window which defines target reached.");
			Serial.println(" ");
			Serial.println("emergency stop is ES#");

			// emergency stop is ES#


		}


		if (receivedData.startsWith("ES", 0))               // Emergency stop requested from C# driver
		{

			stepper.stop();
			SlewStatus = false;
			DecelFlag = false;
		}                                                   // end Emergency Stop else clause



		//*************************************************************************
		//******** code the SA process below **************************************
		//**** example of data sent by driver SA220.00#  **************************
		//**** SA command is followed in the driver by sending CL# or CC# *********
		//*************************************************************************

		if (receivedData.startsWith("SA", 0)) // SlewToAzimuth command from C# driver
		{
			char message;                         //used to hold the char value of TargetAzimuth
			//Serial.println(" 2 received SA   " + receivedData);   //TEST ONLY REMOVE
			// strip off 1st 2 chars
			receivedData.remove(0, 2);


			TargetAzimuth = receivedData.toFloat();    // store the target az for comparison with current position
			receivedData = "";
			// dtostrf(TargetAzimuth, 7, 2, message); // convert double to char total width 7 with 2 dp
			//String lcdazimuth = String(message);
			// write the target azimuth to the LCD screen

			//      Serial.println(" 3 received SA   " + receivedData);   //TEST ONLY REMOVE
			//	lcd.setCursor(0, 0);
			//	lcd.print("Target Azimuth :");
			//	lcd.setCursor(1,1);
			//	lcd.print(lcdazimuth);
			//Serial.println("received SA   " + receivedData);
			//  Serial.print(" 4 lcdazimuth ");
			//  Serial.println(lcdazimuth);
			stepper.setCurrentPosition(1);      // new in v4
			//Serial.println(" EXIT SA  " );
		}



		if (receivedData.startsWith("CL", 0))        //   clockwise Slew command from C# driver
		{
			//Serial.println("moving CLockwise START CL  " );
			//Serial.println("moving CLockwise   " + receivedData);
			stepper.moveTo(30000);                      //  Negative is anticlockwise pos is clockwise from the 0 position.

			// used 30000 as one full rev of dome and this should therefore cover any size slew

			stepper.setMaxSpeed(StepsPerSecond);           //  must call this following moveto
			stepper.setAcceleration(normalAcceleration);
			SlewStatus = true;
			Clockwise = true;                           // used for deceleration

			stepper.run();
			receivedData = "";
			//write the direction to the LCD screen
			//Serial.println("moving CLockwise EXIT CL  " );

		} // end if cl

		if (receivedData.startsWith("CC", 0)) //  counter clockwise Slew command from C# driver
		{

			//Serial.println("moving counter clockwise   " + receivedData); C
			stepper.moveTo(-30000);                      //  Negative is anticlockwise pos is clockwise from the 0 position.
			stepper.setMaxSpeed(StepsPerSecond);           //  must call this following moveto
			stepper.setAcceleration(normalAcceleration);
			SlewStatus = true;
			Clockwise = false;                           // used for deceleration

			stepper.run();
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
			//Serial.println("received SL   " + receivedData);
			//

			// strip off 1st 2 chars
			receivedData.remove(0, 2);


			CurrentAzimuth = receivedData.toFloat();    // store the target az for comparison with current position
			receivedData = "";
			// decelerate if within 20 degrees to prevent overshoot
			//


			if ((abs(CurrentAzimuth - TargetAzimuth) < 20.0) && (DecelFlag == false))  // within 20 degrees of target....
			{
				Serial.print("within 20 deg  ");
				DecelFlag = true;                       // set the flag so this code is only executed once
				if (Clockwise)
				{
					stepper.setMaxSpeed(StepsPerSecond * 0.5); // reduce speed to one HALF - clockwise dir
					stepper.setAcceleration(normalAcceleration * 2);
					stepper.run();
				}
				else                                    // else clause is counterclockwise movement of stepper
				{
					stepper.setMaxSpeed(StepsPerSecond * 0.5); // reduce speed to one HALF - anticlockwise dir
					stepper.setAcceleration(normalAcceleration * 2);
					stepper.run();
				}
			}



			// now code cases 1 and 2 from arduino stepper process in spreadsheet
			// which compare currentazimuth with targetazimuth

			// 1 current az and target az are within 5 degrees
			//
			if (abs(CurrentAzimuth - TargetAzimuth) < 5.0)
			{



				// new code for v4
				if (Clockwise)
				{
					// Serial.println(stepper.currentPosition());                // FOR TESTING
					stepper.moveTo(stepper.currentPosition() + 100);             // set the end point so deceleration can happen
					// Serial.println(stepper.currentPosition());                // FOR TESTING
				}
				else                                    // else clause is counterclockwise movement of stepper
				{
					stepper.moveTo(stepper.currentPosition() - 100);
				}

				// new code end v4




			}  // end true case

			else // if the two angles are not within 5 deg then the motor is still going

			{
				SlewStatus = true;
				Serial.print("Moving");               // sent to serial USB and picked up by the driver
				Serial.println("#");

				//  Serial.println("Target Az is " + String(TargetAzimuth,2));
				//  Serial.println("current Az is " + String(CurrentAzimuth,2));


			} // end false case

			//write the slew status to the LCD screen

		}  // end SL case



	}  // end software serial
	if (SlewStatus)
	{
		stepper.run();

		if (abs(stepper.distanceToGo()) < 2)
		{
			SlewStatus = false;
			DecelFlag = false;
			stepper.setAcceleration(normalAcceleration);
			Serial.print("Notmoving");                   // sent to serial USB and picked up by the driver
			Serial.println("#");
			//Serial.println("stopped stepper");
		}



		// Serial.println(stepper.currentPosition());
	}
	//Serial.print("TargetAzimuth = " );
	//Serial.print ( CurrentAzimuth);
	//Serial.println();
} // end void
