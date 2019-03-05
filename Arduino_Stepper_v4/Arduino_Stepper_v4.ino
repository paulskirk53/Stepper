// things to try to stop the pause - caused by sending anything# over serial monitor
// speed up serial - THIS SEEMS  to have helped - field try necessary
// don't use floating point - could use integer angles without loss really
// use a teensy board instead
// if TargetAz < min or targetaz > max need to do something to avoid collision at pulley//
// Calculations for the stepper with the driver set to 0.25 degree steps show that 35000 (thirty five thousand)
// steps are necessary to pull in enough cord to do a half rotation of the dome. So the figure of 30,000 set when the software
// was written initially had to be increased to a bigger number - it's now set at 300,000 which is way bigger than any slew would need.
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

int normalAcceleration;
int lower_limit = 0;
int upper_limit = 360;
long pkinterval = 0;
long pkstart = 0;
long pkfinish = 0;

/*
--------------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------------
*/


void setup()
{
	// put your setup code here, to run once:

	Serial.begin(115200) ;      //230400);                           // start serial ports - usb with PC
	stepper.stop();                               // set initial state as stopped
	// Change below to suit the stepper
	
	SlewStatus = false;
	StepsPerSecond = 500.0;                       // changed following empirical testing
	normalAcceleration = 50;                       // changed following empirical testing
	stepper.setMaxSpeed(StepsPerSecond);          // steps per second see below -
	// the controller electronics is set to 0.25 degree steps, so 15 stepspersecond*0.25= 3.75 degrees of shaft movement per second
	stepper.setAcceleration(normalAcceleration);  // steps per second per second.
	// Note V= acceleration * time, so a vlue of e.g. 1 step /s/s takes 10 secs to reach maxspeed of 10 or 15 secs to reach maxspeed 15 etc
	//
	// see how the speed of 15 goes empirically
	// with a 10 cm diameter drive wheel on the shaft, calculations show 20 shaft rotations
	// are required to move the dome through approx one revolution and this works out to be 27600 steps for one dome rotation

	

	// initialise slewtoAz, currentazimuth
	TargetAzimuth =  0.0;
	CurrentAzimuth = 0.0;
	

	/*
	default pos for MA860H controller switches:
	5     6    7    8
	on   off  off  off = smallest step size

	my setting:
	5     6    7    8
	on   on  off  on =  step angle ~ 0.25 degree

	*/

	lcd.begin(20, 4);                      // 20 columns x 4 rows
	lcd.clear();

	endpointdone = false;                  // to facilitate one time execution of the endpoint setting when within 5 degrees of target


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

	if (Serial.available() > 0)                              // when serial data arrives capture it into a string
	{

		receivedData = Serial.readStringUntil('#');          // read a string from PC serial port usb



		if (receivedData.startsWith("TEST", 0))
		{
			lcd.setCursor(0, 0);
			lcd.print("Test Received");
			delay(1000);
			lcd.clear();
			Serial.println("Communications established, received  " + receivedData);
			Serial.println("Target Az   " +  String( TargetAzimuth));
			Serial.println("Current Az   " + String( CurrentAzimuth));

			Serial.println(" so a good sequence would be as follows IN THE ORDER RECEIVED FROM the driver....");
			Serial.println("1 CL190#   clockwise movement request with current Az from encoder");
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
		    lcd.clear();
			Emergency_Stop(TargetAzimuth, "Received ES");
		}                                                   // end Emergency Stop else clause



		//*************************************************************************
		//******** code for SA process below **************************************
		//**** example of data sent by driver SA220.00#  **************************
		//**** SA command is followed in the driver by sending CL# or CC# *********
		//*************************************************************************

		if (receivedData.startsWith("SA", 0)) // SlewToAzimuth command from C# driver
		{
			
			
			// strip off 1st 2 chars
			receivedData.remove(0, 2);


			TargetAzimuth = receivedData.toFloat();    // store the target az for comparison with current position
			// if target < min or target > max need to do something to avoid collision at pulley

			SlewStatus = true;
			

			lcdprint(0, 0,"Az requested");
			lcdprint(13,0,receivedData);


			if ((TargetAzimuth < lower_limit ) || (TargetAzimuth > upper_limit))   //error trap azimuth value
			{
				Emergency_Stop(TargetAzimuth, "Target Az failure   ");
			}
			

			receivedData = "";
			
		}



		if (receivedData.startsWith("CL", 0))            // clockwise Slew command from C# driver
		{

			receivedData.remove(0, 2);                      // strip off 1st 2 chars
			CurrentAzimuth = receivedData.toFloat();        // store the current az for comparison with current position

			// used 30000 as one full rev of dome and this should therefore cover any size slew
			stepper.setMaxSpeed(1);
			stepper.setMaxSpeed(StepsPerSecond);         // must call this following moveto
			stepper.setAcceleration(normalAcceleration);
			
			Clockwise = true;                            // used for deceleration
			
			
			stepper.setCurrentPosition(15)  ;            // outside the aceel/ decel range checker
			stepper.moveTo(100000);                      // Negative is anticlockwise pos is clockwise from the 0 position.
			stepper.run();
			lcdprint(0, 2,"Clockwise direction");
			
			receivedData = "";


		} // end if cl

		if (receivedData.startsWith("CC", 0))            //  counter clockwise Slew command from C# driver
		{
			receivedData.remove(0, 2);                   // strip off 1st 2 chars
			CurrentAzimuth = receivedData.toFloat();     // store the current az for comparison with current position

			stepper.setMaxSpeed(StepsPerSecond);         // must call this following moveto
			stepper.setAcceleration(normalAcceleration);
			
			Clockwise = false;                           // used for deceleration
			
			stepper.setCurrentPosition(-15)    ;         // outside the aceel/ decel range checker
			stepper.moveTo(-100000);                     // Negative is anticlockwise pos is clockwise from the 0 position.
			stepper.run();
			
			lcdprint(0, 2,"Anticlockwise Dir");
			
			receivedData = "";
			//write the direction to the LCD screen

		} // end if cc



		//*************************************************************************
		// ******** code for SL process below *************************************
		//**** example of data sent by driver SL220.00#  **************************
		//*************************************************************************
		//

		if (receivedData.startsWith("SL", 0)) // Receive the Current AZ via driver from Compass
		{

			// strip off 1st 2 chars
			receivedData.remove(0, 2);


			CurrentAzimuth = receivedData.toFloat();    // store the target az for comparison with current position

			//PRINT TO LCD
			lcdprint(0, 1,"Current Az ");
			lcdprint(13,1,receivedData);

			if ((CurrentAzimuth < lower_limit ) || (CurrentAzimuth > upper_limit))   //error trap azimuth value
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

		
		
		within_five_degrees();                  

		 // Check how close to the endpoint and reset flags for motor stop and initialisation of variables

		//new  - used to be a routine called distancechecker, but for some reason unknown after investigation, it added 30ms to void loop
		//so I removed the call and replaced it with the code.

		if (abs(stepper.distanceToGo()) < 5)                     // within 5 steps of target
		{
			stepper.stop();
			SlewStatus = false;                             // used to stop the motor in main loop
			
			endpointdone = false;                           // RESET this so that the 5 degree window for stopping is enabled
			
			lcdprint(0, 2,"Movement Stopped.   ");
			lcdprint(0, 4,"Target achieved.    ");
		}


		//end new
						

		pkfinish= millis();
		pkinterval= pkfinish - pkstart;

		// set trace on stepper,run below : {stepper.distanceToGo()}{stepper.currentPosition()}{stepper.speed()}

		stepper.run();
	
	}

	//stepper.run();

} // end void Loop



void within_five_degrees()
{

	// now code cases 1 and 2 from arduino stepper process in spreadsheet
	// which compare currentazimuth with targetazimuth

	// 1 current az and target az are within 5 degrees
	//

	if ((abs(CurrentAzimuth - TargetAzimuth) < 5.0  ) && (endpointdone == false))
	{
	    lcdprint(0,4,"Slowing to target");
		endpointdone=true;
		
		stepper.setAcceleration(normalAcceleration*2);                   // change acceleration here if we want a different rate of deceleration
		if (Clockwise)
		{
			stepper.moveTo(stepper.currentPosition() + 2000);           // set the end point so deceleration can happen - 360 matches the maxspeed
		}
		else                                                          //  else clause is counterclockwise movement of stepper
		{
			stepper.moveTo(stepper.currentPosition() - 2000);
		}

	}  // end true case

	// if the two angles are not within 5 deg then the motor is still going


}


void Emergency_Stop(double azimuth, String mess)
{

	stepper.stop();
	SlewStatus = false;
	endpointdone = false;

	lcdprint(0,0,"Stopped");
	lcdprint(0, 1,mess);
	
	
}
void lcdprint(int col, int row, String mess)
{
	//lcd.clear();
	lcd.setCursor(col, row);
	lcd.print(mess);

}
