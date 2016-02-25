

#ifndef _DEFS_h
#define _DEFS_h

// these are not constants because they can be 
// changed on the fly through the serial port
double P_A = 0.005;
double I_A = 0.009;
double D_A = 0.006; //small because we have small resolution (200 samples)

/* 
The altitude readings need to be adjusted, they are the potentiometer's max and min
altitude readings so we have a reference point for what location we are at and can
scale the input to within 0-255 for the PID library
*/
#define MIN_ALTITUDE_READING			171.0
#define MAX_ALTITUDE_READING			450.0

#define MIN_ALTITUDE_OUTPUT				210	// around halfway it will go backwards, never let it go backwards
#define MAX_ALTITUDE_OUTPUT				254	// after 255 it will overflow 

#define INITIAL_ALTITUDE_SETPOINT		150
#define INITIAL_ALTITUDE_OUTPUT			210

#define ALTITUDE_INPUT_PIN				3
#define ALTITUDE_OUTPUT_PIN				3  

#endif

