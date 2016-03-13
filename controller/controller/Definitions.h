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
#define P_A		0.005
#define I_A		0.009
#define D_A		0.005 //small because we have small resolution (200 samples)

// yaw
#define P_Y		0.100
#define I_Y		0.050
#define D_Y		0.210

#define ALTITUDE_INPUT_PIN				1
#define ALTITUDE_OUTPUT_PIN				9

#define YAW_INPUT_PIN_1					2
#define YAW_INPUT_PIN_2					3
#define YAW_OUTPUT_PIN					10

#define ALTITUDE_RANGE					200
#define NUMBER_OF_SAMPLES_RECORDED		100

#define YAW_TRIAL_OFFSET				200

#define YAW_KD_INCREMENT				0.01
#define YAW_KI_INCREMENT				0.01
#define YAW_KP_INCREMENT				0.01

#define YAW_NUMBER_OF_ITERATIONS_EACH	3		

#endif

