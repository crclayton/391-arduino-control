/*
* Author: Charles Clayton
* Date: 23-Feb-16
* Description: PID control for the lever arm altitude
* Additional info:
*    Arduino PMW output signal for PIN 3 is 16MHz / 64 / 255 / 2 = 490.196Hz -> 2.041ms T
*/

#include <PID_v1.h>

/* THESE NEED TO BE ADJUSTED UPON STARTUP :S */
#define MIN_ALTITUDE_READING             427.0
#define MAX_ALTITUDE_READING             638.0

/*  The above readings are scaled between 0-255, and the
angle setpoint is the desired altitude between 0-255, usually halfway  */
#define ANGLE_SETPOINT                0.5 * 255 

#define P_A   0.005  
#define I_A   0.009
#define D_A   0.005 // small because we have small resolution (200 samples)

#define ALTITUDE_INPUT_PIN       3
#define ALTITUDE_OUTPUT_PIN           3  


double pidAltitudeSetpoint,
pidAltitudeInput,
pidAltitudeOutput;	

PID altitudePID(
	&pidAltitudeInput,     /* actual value */
	&pidAltitudeOutput,    /* modification value, 0-255 */
	&pidAltitudeSetpoint,  /* desired value */
	P_A, I_A, D_A, DIRECT);

void setup()
{
	pinMode(ALTITUDE_OUTPUT_PIN, OUTPUT);
	pinMode(ALTITUDE_INPUT_PIN, INPUT);

	Serial.begin(9600);

	// 255   = 100% duty cycle, 1/490Hz    -> 2.041ms T - fastest
	// 127.5 = 50%  duty cycle, 0.75/490Hz -> 1.531ms T - almost 0
	altitudePID.SetOutputLimits(200, 250);
	altitudePID.SetMode(AUTOMATIC);

	/*set initial values*/
	analogWrite(ALTITUDE_OUTPUT_PIN, 200);
	pidAltitudeSetpoint = ANGLE_SETPOINT;
}

void loop()
{
	// setpoint driven w/ PID
	double raw_altitude = analogRead(ALTITUDE_INPUT_PIN);
	// current altitude scaled between 0-255
	pidAltitudeInput = scaleBetween(raw_altitude, MIN_ALTITUDE_READING, MAX_ALTITUDE_READING, 0.0, 255.0);

	altitudePID.Compute();
	analogWrite(ALTITUDE_OUTPUT_PIN, pidAltitudeOutput);

	Serial.print("Setpoint: ");
	Serial.print(pidAltitudeSetpoint);
	Serial.print(",\tRaw Input:");
	Serial.print(raw_altitude);
	Serial.print(",\tScaled Input: ");
	Serial.print(pidAltitudeInput);
	Serial.print(",\tDiff: ");
	Serial.print(percentDifference(pidAltitudeInput, pidAltitudeSetpoint));
	Serial.print(",\tOutput: ");
	Serial.print(pidAltitudeOutput);
	Serial.print("\r\n");

	// change altitude setpoint if new value given
	if (Serial.available() > 0) {
		pidAltitudeSetpoint = Serial.parseFloat();
	}
}

double scaleBetween(double input, double minimum, double maximum, double output_min, double output_max) {
	return (((input - minimum) * (output_max - output_min)) / (maximum - minimum)) + output_min;
}

double percentDifference(double input, double setpoint) {
	return (setpoint - input) / ((setpoint + input) / 2) * 100;
}