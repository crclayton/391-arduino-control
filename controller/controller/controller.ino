/*
* Author: Charles Clayton
* Date: 23-Feb-16
* Description: PID control for the lever arm altitude
* Additional info:
*    Arduino PMW output signal for PIN 3 is 16MHz / 64 / 255 / 2 = 490.196Hz -> 2.041ms T
*/

#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <Defs.h>

double pidAltitudeSetpoint,
	pidAltitudeInput,
	pidAltitudeOutput;	

PID altitudePID(
	&pidAltitudeInput,     /* actual value */
	&pidAltitudeOutput,    /* modification value, 0-255 */
	&pidAltitudeSetpoint,  /* desired value */
	P_A, I_A, D_A, DIRECT);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

bool buttonMinMode = true;

void setup()
{
	pinMode(ALTITUDE_OUTPUT_PIN, OUTPUT);
	pinMode(ALTITUDE_INPUT_PIN, INPUT);

	Serial.begin(9600);

	// define number of columns and rows
	lcd.begin(20, 4);
	lcd.print("Team A2 TTv1");
	// 255   = 100% duty cycle, 1/490Hz    -> 2.041ms T - fastest
	// 127.5 = 50%  duty cycle, 0.75/490Hz -> 1.531ms T - almost 0
	// we give it 200 and 250 so it will never go in reverse and 
	// won't go full-speed ahead
	altitudePID.SetOutputLimits(MIN_ALTITUDE_OUTPUT, MAX_ALTITUDE_OUTPUT);
	altitudePID.SetMode(AUTOMATIC);
	altitudePID.SetSampleTime(100); // instead of 100ms, recalculate every 10ms

	/*set initial values*/
	//analogWrite(ALTITUDE_OUTPUT_PIN, INITIAL_ALTITUDE_OUTPUT);
	pidAltitudeSetpoint = INITIAL_ALTITUDE_SETPOINT;
}

void loop()
{
	// set-point driven w/ PID
	double raw_altitude = analogRead(ALTITUDE_INPUT_PIN);
	// current altitude scaled between 0-255
	pidAltitudeInput = scaleValue(raw_altitude, MIN_ALTITUDE_READING, MAX_ALTITUDE_READING, 0.0, 255.0);

	// if the button is pressed
	if (buttonPressed(BUTTON_INPUT_PIN)) {
		//and we are in minimum mode, zero the min altutide reading
		if (buttonMinMode)	MIN_ALTITUDE_READING = raw_altitude;
		else                MAX_ALTITUDE_READING = raw_altitude;

		// then change the mode we're in NOTE: THIS MIGHT NOT WORK
		buttonMinMode = !buttonMinMode;
	}

	altitudePID.Compute();
	analogWrite(ALTITUDE_OUTPUT_PIN, pidAltitudeOutput);

	lcdPrint("Height: ", pidAltitudeInput, 2, 1);
	lcdPrint("", raw_altitude, 2, 16);
	lcdPrint("Target: ", pidAltitudeSetpoint, 3, 1);
	lcdPrint("Output: ", pidAltitudeOutput, 4, 1);

	// change altitude set-point if new value given
	if (Serial.available() > 0) {
		assignSerialInput(Serial.readString());
	}
}						  

int assignSerialInput(String serialInput) {

	if (serialInput[0] != '-') {
		Serial.println("ERROR: Expecting identifier. Ex: '-P 0.005' or '-s 150'");
		return -1;
	}

	char serialInputIdentifier = serialInput[1];
	double serialInputValue = serialInput.substring(3).toFloat();

	switch (serialInputIdentifier) {
		case('s') :
			pidAltitudeSetpoint = serialInputValue;
			break;
		case('o') :
			pidAltitudeOutput = serialInputValue;
			break;
		case('P') :
			P_A = serialInputValue;
			altitudePID.SetTunings(P_A, I_A, D_A);
			break;
		case('I') :
			I_A = serialInputValue;
			altitudePID.SetTunings(P_A, I_A, D_A);
			break;
		case('D') :
			D_A = serialInputValue;
			altitudePID.SetTunings(P_A, I_A, D_A);
		default:
			Serial.println("ERROR: Unknown identifier.");
			return -1;
	}

	Serial.print("SUCCESS: Value ");
	Serial.print(serialInputValue);
	Serial.print(" was assigned to ");
	Serial.print(serialInputIdentifier);
	Serial.print("\r\n");
	return 0;
}

void lcdPrint(String label, double value, int row, int column) {
	lcd.setCursor(column - 1, row - 1); // -1 so we can use natural numbers
	lcd.print(label + value); //label.concat(value));
}

double scaleValue(double input, double minimum, double maximum, double output_min, double output_max) {
	return (((input - minimum) * (output_max - output_min)) / (maximum - minimum)) + output_min;
}

double percentDifference(double input, double setpoint) {
	return (setpoint - input) / ((setpoint + input) / 2) * 100;
}

bool buttonPressed(int pin) {
	int buttonState = digitalRead(pin);
	return buttonState == HIGH;
}