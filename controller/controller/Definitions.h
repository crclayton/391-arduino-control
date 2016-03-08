// more voltage, more resolution!!

#ifndef _DEFS_h
#define _DEFS_h

// these are not constants because they can be 
// changed on the fly through the serial port
double P_A = 0.005;
double I_A = 0.009;
double D_A = 0.005; //small because we have small resolution (200 samples)

double P_Y = 0.005;
double I_Y = 0.009;
double D_Y = 0.005; //small because we have small resolution (200 samples)

double MIN_ALTITUDE_READING = 288.0;
double MAX_ALTITUDE_READING = 130.0;

double MAX_YAW_READING = 10.0;
double MIN_YAW_READING = 800.0;

#define ALTITUDE_INPUT_PIN				1
#define ALTITUDE_OUTPUT_PIN				3  

#define YAW_INPUT_PIN					5
#define YAW_OUTPUT_PIN					9

//#define BUTTON_INPUT_PIN				2

/* 
The altitude readings need to be adjusted, they are the potentiometer's max and min
altitude readings so we have a reference point for what location we are at and can
scale the input to within 0-255 for the PID library
*/

#define MIN_ALTITUDE_OUTPUT				190	// around halfway it will go backwards, never let it go backwards
#define MAX_ALTITUDE_OUTPUT				254	// after 255 it will overflow 


#define MIN_YAW_OUTPUT				120	
#define MAX_YAW_OUTPUT				200	

#define INITIAL_YAW_OUTPUT			155
#define INITIAL_ALTITUDE_OUTPUT		155

#endif
