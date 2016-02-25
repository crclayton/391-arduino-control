/*
* Author: Charles Clayton
* Date: 23-Feb-16
* Description: PID control for the lever arm altitude
* Additional info:
*    Arduino PMW output signal for PIN 3 is 16MHz / 64 / 255 / 2 = 490.196Hz -> 2.041ms T
*/

#include <PID_v1.h>

/* THESE NEED TO BE ADJUSTED UPON STARTUP :S */
#define MIN_ANGLE_READING             427.0
#define MAX_ANGLE_READING             638.0

/*  The above readings are scaled between 0-255, and the
angle setpoint is the desired altitude between 0-255, usually halfway  */
#define ANGLE_SETPOINT                0.5 * 255 

#define P   0.005  
#define I   0.009
#define D   0.005 // small because we have small resolution (200 samples)

#define POTENTIOMETER_INPUT_PIN       3
#define PID_OUTPUT_PIN                3  


double pid_setpoint, pid_input, pid_output,
potentiometer_input, resistor_input;


PID myPID(
	&pid_input,     /* actual value */
	&pid_output,    /* modification value, 0-255 */
	&pid_setpoint,  /* desired value */
	P, I, D, DIRECT);

void setup()
{
	pinMode(PID_OUTPUT_PIN, OUTPUT);
	pinMode(POTENTIOMETER_INPUT_PIN, INPUT);

	Serial.begin(9600);

	// 255   = 100% duty cycle, 1/490Hz    -> 2.041ms T - fastest
	// 127.5 = 50%  duty cycle, 0.75/490Hz -> 1.531ms T - almost 0
	myPID.SetOutputLimits(200, 250);
	myPID.SetMode(AUTOMATIC);

	/*set initial values*/
	analogWrite(PID_OUTPUT_PIN, 200);
	pid_setpoint = ANGLE_SETPOINT;
}

void loop()
{
	// setpoint driven w/ PID
	double raw_input = analogRead(POTENTIOMETER_INPUT_PIN);

	// current altitude scaled between 0-255
	pid_input = scaleBetween(raw_input, MIN_ANGLE_READING, MAX_ANGLE_READING, 0.0, 255.0);

	myPID.Compute();
	analogWrite(PID_OUTPUT_PIN, pid_output);

	Serial.print("Setpoint: ");
	Serial.print(pid_setpoint);
	Serial.print(",\tRaw Input:");
	Serial.print(raw_input);
	Serial.print(",\tScaled Input: ");
	Serial.print(pid_input);
	Serial.print(",\tDiff: ");
	Serial.print(percentDifference(pid_input, pid_setpoint));
	Serial.print(",\tOutput: ");
	Serial.print(pid_output);
	Serial.print("\r\n");

	// change altitude setpoint if value given
	if (Serial.available() > 0) {
		pid_setpoint = Serial.parseFloat();
	}
}

double scaleBetween(double input, double minimum, double maximum, double output_min, double output_max) {
	return (((input - minimum) * (output_max - output_min)) / (maximum - minimum)) + output_min;
}

double percentDifference(double input, double setpoint) {
	return (setpoint - input) / ((setpoint + input) / 2) * 100;
}