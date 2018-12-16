// Program writen by Paul Kirk to handle Dome driver receipts December 2017
//now using visual micro in atmel studio for development 30-5-2018
// Boiler PC Vmicro menu -> build, and build and upload work and tested on a spare Uno 30-5-18, with the sequence in section dated 29-5-18 below
// the TEST# function has been coded for testing purposes (same on Compass sketch); it writes to the serial monitor.

// 29-7-18 The drive for the dome has changed from a friction device which allows multiple 360 degree rotations in any direction
// to a pull cord arrangement which has a defined start and end point (which may be the same if the mechanics work out).
// If the start and end point can be the same, then the limitation is that 360 degree point cannot be crossed, so if the system is at say 350 degrees
// and a request comes in for 10 degrees, the system has to go backwards 360 ==> 10 to get there.
// it may be best to handle this in the driver rather than in this code - needs investigation.



// 29-5-18 - for physical tests send SA220# or somesuch followed by CC# or CL#
// then immediately send SL240# which simulates the driver sending the compass value to this program.

// so a good sequence would be 
// 1 SA220# 
// 2 CL# 
// 3 SL180#
// 4 SL200#               tests the within 20 degrees bit which should reduce speed to one third
// 5 SL118#               should stop when this is entered because we are within the coded 5 degree window which defines target reached.

// emergency stop is ES#
//
// Define South? presumabaly in the compass module


//  VERSION 2 changes to look if position is within 20 degrees of target and slows the stepper to avoid overshoot
//            Changed from use of stepper.run to stepper.runSpeed because stepper.setspeed works with that command
//       
//
// Handles the following processes and properties
//  SA - Slew to Azimuth
//  SL - Slew Status 
//  CC move counter clockwise
//  CL move clockwise
//  code informed by google sheet - 'Dome driver program process'
//  COM4 whilst testing on boiler pc

      //*****************************************************************************************
      //************************ known potential problems  **************************************
      //**** Stepper.moveto uses a value of 10000 and for large moves in R.A. *******************
      //**** this value may allow the stepper to stop before it reaches the target RA  **********
      //***** in which case the value will need to be larger ************************************
      //*****************************************************************************************

     //*************************************************************************
      //************************ testing  **************************************
      //**** Initial tests on 9-1-18 look good *********************************
      //**** the stepper slows to half speed when targeted in either ***********
      //**** Clockwise or Counterclockwise directions  *************************
      //*****                                                *******************
      //************************************************************************
// bug fix - DecelFlag not reset when the stepper stops, so the deceleration only happened once. 
// fix applied 14-1-18
