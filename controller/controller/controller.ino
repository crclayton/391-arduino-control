/*
* Author: Charles Clayton
* Date: 23-Feb-16
* Description: PID control for the lever arm altitude
* Additional info:
*    Arduino PMW output signal for PIN 3 is 16MHz / 64 / 255 / 2 = 490.196Hz -> 2.041ms T
*/

#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <Definitions.h>

double pidAltitudeSetpoint,
	pidAltitudeInput,
	pidAltitudeOutput;

double pidYawSetpoint,
	pidYawInput,
	pidYawOutput;

volatile int raw_yaw = 0;

PID altitudePID(
	&pidAltitudeInput,     /* actual value */
	&pidAltitudeOutput,    /* modification value, 0-255 */
	&pidAltitudeSetpoint,  /* desired value */
	P_A, I_A, D_A, DIRECT);

PID yawPID(
	&pidYawInput,     /* actual value */
	&pidYawOutput,    /* modification value, 0-255 */
	&pidYawSetpoint,  /* desired value */
	P_Y, I_Y, D_Y, DIRECT);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

bool buttonMinMode = true;

void setup()
{
	pinMode(ALTITUDE_OUTPUT_PIN, OUTPUT);
	pinMode(ALTITUDE_INPUT_PIN, INPUT);

	pinMode(YAW_OUTPUT_PIN, OUTPUT);
	pinMode(YAW_INPUT_PIN_1, INPUT);
	pinMode(YAW_INPUT_PIN_2, INPUT);

	attachInterrupt(0, doEncoderA, CHANGE);
	attachInterrupt(1, doEncoderB, CHANGE);

	Serial.begin(9600);

	// define number of columns and rows
	lcd.begin(20, 4);
	// 255   = 100% duty cycle, 1/490Hz    -> 2.041ms T - fastest
	// 127.5 = 50%  duty cycle, 0.75/490Hz -> 1.531ms T - almost 0
	// we give it 200 and 250 so it will never go in reverse and 
	// won't go full-speed ahead
	altitudePID.SetOutputLimits(MIN_ALTITUDE_OUTPUT, MAX_ALTITUDE_OUTPUT);
	altitudePID.SetMode(AUTOMATIC);
	altitudePID.SetSampleTime(100); // instead of 100ms, recalculate every 10ms

	yawPID.SetOutputLimits(MIN_YAW_OUTPUT, MAX_YAW_OUTPUT);
	yawPID.SetMode(AUTOMATIC);
	yawPID.SetSampleTime(100); // instead of 100ms, recalculate every 10ms

	/*
	// set setpoints to halfway between the min and max readings
	pidAltitudeSetpoint = abs(MAX_ALTITUDE_READING - MIN_ALTITUDE_READING)/2.0;
	pidYawSetpoint = abs(MAX_YAW_READING - MIN_YAW_READING) / 2.0;
	*/

	// set setpoints to current positions until specified
	pidAltitudeSetpoint = scaleValue(
		analogRead(ALTITUDE_INPUT_PIN), 
		MIN_ALTITUDE_READING, MAX_ALTITUDE_READING, 0.0, 255.0);

	pidYawSetpoint = scaleValue(
		raw_yaw, 
		MIN_YAW_READING, MAX_YAW_READING, 0.0, 255.0);


}

bool first = true;

void loop()
{
	// hackery alert. 
	// The inital output is set to the minimum output limit
	// at first, that's fine with the altitude motor because it 
	// goes only in one direction,  so we set it the lower limit
	// to not moving because it only needs to move one direction

	// the yaw needs to move two directions, so the lower limit needs to be the
	// maximum velocity in the negative direction so upon startup that's what it
	// does, not good. So what we do is we originally set the minimum to be not moving,
	// then after that's what the initalized, we reduce the lower output limit
	if (first) {
		yawPID.SetOutputLimits(160, MAX_YAW_OUTPUT);
		first = false;
	}

	// set-point driven w/ PID
	double raw_altitude = analogRead(ALTITUDE_INPUT_PIN);

	// current altitude scaled between 0-255
	pidAltitudeInput = scaleValue(raw_altitude, MIN_ALTITUDE_READING, MAX_ALTITUDE_READING, 0.0, 255.0);
	pidYawInput = scaleValue(raw_yaw, MIN_YAW_READING, MAX_YAW_READING, 0.0, 255.0);

	altitudePID.Compute();
	yawPID.Compute();

	analogWrite(ALTITUDE_OUTPUT_PIN, pidAltitudeOutput);
	analogWrite(YAW_OUTPUT_PIN, pidYawOutput);

	Serial.print("Altitude I\\S\\O:\t");
	Serial.print(raw_altitude);
	Serial.print("\\");
	Serial.print(pidAltitudeInput);
	Serial.print("\\");
	Serial.print(pidAltitudeSetpoint);
	Serial.print("\\");
	Serial.print(pidAltitudeOutput);

	Serial.print("\tYaw I\\S\\O:\t");
	Serial.print(pidYawInput);
	Serial.print("\\");
	Serial.print(pidYawSetpoint);
	Serial.print("\\");
	Serial.print(pidYawOutput);
	
	Serial.print("\n\r");

	lcd.setCursor(0, 0); 
	lcd.print("   Altitude  Yaw");
	lcdPrint("I: ", pidAltitudeInput, 2, 1);
	lcdPrint("S: ", pidAltitudeSetpoint, 3, 1);
	lcdPrint("O: ", pidAltitudeOutput, 4, 1);

	lcdPrint("", pidYawInput, 2, 14);
	lcdPrint("", pidYawSetpoint, 3, 14);
	lcdPrint("", pidYawOutput, 4, 14);



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
		case('a') :
			pidAltitudeSetpoint = serialInputValue; break;
		case('A') :
			pidAltitudeOutput = serialInputValue; break;
		case('y') :
			pidYawSetpoint = serialInputValue; break;
		case('Y') :
			pidYawOutput = serialInputValue; break;
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


void doEncoderA() {

	if (digitalRead(YAW_INPUT_PIN_1) == HIGH) {

		if (digitalRead(YAW_INPUT_PIN_2) == LOW) 
			raw_yaw = raw_yaw + 1;         
		else 
			raw_yaw = raw_yaw - 1;         
	}
	else                                      
	{
		if (digitalRead(YAW_INPUT_PIN_2) == HIGH) 
			raw_yaw = raw_yaw + 1;          		
		else 
			raw_yaw = raw_yaw - 1;          
	}
}

void doEncoderB() {

	if (digitalRead(YAW_INPUT_PIN_2) == HIGH) {

		if (digitalRead(YAW_INPUT_PIN_1) == HIGH) 
			raw_yaw = raw_yaw + 1;        
		else 
			raw_yaw = raw_yaw - 1;        
	}
	else {

		if (digitalRead(YAW_INPUT_PIN_1) == LOW) 
			raw_yaw = raw_yaw + 1;          
		else 
			raw_yaw = raw_yaw - 1;         
	}

}
