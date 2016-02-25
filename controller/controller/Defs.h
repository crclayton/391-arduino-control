// Defs.h

#ifndef _DEFS_h
#define _DEFS_h


/* These values need to be adjusted, they are the potentiometer's max and min
altitude readings so we have a reference point for what location we are at and can
scale the input to within 0-255 */
#define MIN_ALTITUDE_READING             427.0
#define MAX_ALTITUDE_READING             638.0

/* The above readings are scaled between 0-255, and the
angle setpoint is the desired altitude between 0-255,
to start with, we set it to halfway */
#define INITIAL_ALTITUDE_SETPOINT                0.5 * 255 

#define P_A   0.005  
#define I_A   0.009
#define D_A   0.005 // small because we have small resolution (200 samples)

#define ALTITUDE_INPUT_PIN		3
#define ALTITUDE_OUTPUT_PIN     3  


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

