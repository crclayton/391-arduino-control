// more voltage, more resolution!!

#ifndef _DEFS_h
#define _DEFS_h


/*	
	Altitude motor:
		Around 180 is spinning but not fast enough to lift it
		Around 210 is spinning at hovering

	Yaw motor:
		180 not moving
		190 moving clockwise (backwards)



*/




// these are not constants because they can be 
// changed on the fly through the serial port

// altitude
double P_A = 0.005;
double I_A = 0.009;
double D_A = 0.005; //small because we have small resolution (200 samples)

// yaw
double P_Y = 0.100;
double I_Y = 0.050;
double D_Y = 0.200;

double MIN_ALTITUDE_READING = 623.0;
double MAX_ALTITUDE_READING = 480.0;

#define MIN_YAW_READING					200
#define MAX_YAW_READING					-200

#define ALTITUDE_INPUT_PIN				1
#define ALTITUDE_OUTPUT_PIN				9

#define YAW_INPUT_PIN_1					2
#define YAW_INPUT_PIN_2					3
#define YAW_OUTPUT_PIN					10

#define ALTITUDE_RANGE					200

#endif

