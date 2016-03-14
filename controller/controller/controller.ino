/*
* Author: Charles Clayton
* Date: 23-Feb-16
* Description: PID control for the lever arm altitude
* Additional info:
*    Arduino PMW output signal for PIN 3 is 16MHz / 64 / 255 / 2 = 490.196Hz -> 2.041ms T
*/

#include <LiquidCrystal.h>
#include <PID_v1_trimmed.h>
#include <Definitions.h>

class ControlDirection {
public:
	double setpoint;
	double input;
	double output;
	double rawInput;

	// these are needed to scale the input
	// between 0-255 for processing
	double minInput;
	double maxInput;

	// these are needed to scale output
	// duty cycle between 
	double minOutput;
	double maxOutput;

	double KP;
	double KI;
	double KD;

	long unsigned int errorTotal;
	double sampleRecord[NUMBER_OF_SAMPLES_RECORDED];

	void setPidGains(double kp, double ki, double kd ) {
		KP = kp;
		KI = ki;
		KD = kd;
	};
};

ControlDirection Yaw, Altitude;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);


PID AltitudePid(
	&Altitude.input,     /* actual value */
	&Altitude.output,    /* modification value, 0-255 */
	&Altitude.setpoint,  /* desired value */
	P_A, I_A, D_A, DIRECT);

PID YawPid(
	&Yaw.input,     
	&Yaw.output,    
	&Yaw.setpoint,  
	P_Y, I_Y, D_Y, DIRECT);

long unsigned int errorSum = 0;
int sampleNumber = 0;

double startingGains = 0.01;
int iterationsPerGain = 5;
double incrementBy = 0.01;
int trialIteration = 0;

void setup()
{
	// initialize pins and interupts
	pinMode(ALTITUDE_OUTPUT_PIN, OUTPUT);
	pinMode(ALTITUDE_INPUT_PIN, INPUT);

	pinMode(YAW_OUTPUT_PIN, OUTPUT);
	pinMode(YAW_INPUT_PIN_1, INPUT);
	pinMode(YAW_INPUT_PIN_2, INPUT);

	attachInterrupt(0, encoderInteruptLineA, CHANGE);
	attachInterrupt(1, encoderInteruptLineB, CHANGE);

	// configure serial and LCD
	Serial.begin(9600);
	lcd.begin(20, 4); // define number of columns and rows

	//----------- configure PID values ----------- //

	// 255   = 100% duty cycle, 1/490Hz    -> 2.041ms T - fastest
	// 127.5 = 75%  duty cycle, 0.75/490Hz -> 1.531ms T - almost 0
	// we give it 200 and 250 so it will never go in reverse and 
	// won't go full-speed ahead

	// YAW
	//Yaw.setPidGains(0.005, 0.005, 0.005);
	Yaw.maxOutput = 250;
	Yaw.minOutput = 155;
	Yaw.minInput = 200;
	Yaw.maxInput = -200;
	YawPid.SetOutputLimits(Yaw.minOutput, Yaw.maxOutput);
	YawPid.SetMode(AUTOMATIC);
	// set setpoint to 0 (current position)
	Yaw.setpoint = scaleValue(0, Yaw.minInput, Yaw.maxInput, 0.0, 255.0);

	// ALTITUDE
	//Altitude.setPidGains(0.100, 0.050, 0.200);
	Altitude.maxOutput = 254;  
	Altitude.minOutput = 180; // Altitude should never go backwards
	Altitude.rawInput = analogRead(ALTITUDE_INPUT_PIN);
	Altitude.minInput = Altitude.rawInput; // to prevent wander, set minimum to 0 
	Altitude.maxInput = Altitude.rawInput - ALTITUDE_RANGE; // and maximum to 200 points difference
	AltitudePid.SetOutputLimits(Altitude.minOutput, Altitude.maxOutput);
	AltitudePid.SetMode(AUTOMATIC);

	// set setpoints to current position until otherwise specified
	Altitude.setpoint = scaleValue(Altitude.rawInput, 
		Altitude.minInput, Altitude.maxInput, 0.0, 255.0);

}

void loop()
{

	Altitude.rawInput = analogRead(ALTITUDE_INPUT_PIN);

	// current position values from potentiometer scaled between 0-255
	Altitude.input = scaleValue(Altitude.rawInput, 
		Altitude.minInput, Altitude.maxInput, 0.0, 255.0);

	// current position value from encoder's interupt counter
	Yaw.input = scaleValue(Yaw.rawInput, 
		Yaw.minInput, Yaw.maxInput, 0.0, 255.0);

	// calculate new outputs
	AltitudePid.Compute();
	YawPid.Compute();

	// update output 
	analogWrite(ALTITUDE_OUTPUT_PIN, Altitude.output);
	analogWrite(YAW_OUTPUT_PIN, Yaw.output);

	// display all values using LCD 
	lcdPrint("   Altitude  Yaw", 1, 1);
	lcdPrint("I: ", 2, 1);
	lcdPrint("S: ", 3, 1);
	lcdPrint("O: ", 4, 1);

	lcdPrint(String(Altitude.input), 2, 4);
	lcdPrint(String(Altitude.setpoint), 3, 4);
	lcdPrint(String(Altitude.output), 4, 4);

	lcdPrint(String(Yaw.input), 2, 14);
	lcdPrint(String(Yaw.setpoint), 3, 14);
	lcdPrint(String(Yaw.output), 4, 14);

	lcdPrint(String((char)223), 2, 20);
	lcdPrint(String((char)223), 3, 20);

	// if serial input provided, assign to correct value 
	if (Serial.available() > 0) {
		assignSerialInput(Serial.readString());
	}

	// ------------------------------------------------//
	// --------  PID gain experiment trials ---------- //
	// ------------------------------------------------//

	// update sample record with new input
	Yaw.sampleRecord[sampleNumber] = Yaw.input;
	Altitude.sampleRecord[sampleNumber] = Altitude.input;

	// update error sum with new error
	Yaw.errorTotal += abs(Yaw.input - Yaw.setpoint);
	Altitude.errorTotal += abs(Altitude.input - Altitude.setpoint);

	// every 100 samples check if the last 100 samples have all
	// been within the tolerance of the setpoint
	if (sampleNumber > 100) {
		sampleNumber = 0;

		// TODO: include timeout condition in case system goes unstable
		if (eachSampleWithinTolerance(Yaw.sampleRecord, 100, Yaw.setpoint, 0.10) || trialHasTimedOut()) {

			// if they have, print the gains and the total error, 
			// then update the setpoint and change the gains
			Serial.println(
				String(Yaw.KP) + "," + String(Yaw.KI) + "," + 
				String(Yaw.KD) + ":" + String(errorSum));

			Yaw.setpoint = Yaw.setpoint + YAW_TRIAL_OFFSET;
			errorSum = 0;

			// the following code is to iterate through three variables using only one loop
			if (trialIteration % iterationsPerGain == 0) {
				Yaw.KI += incrementBy;
				Yaw.KD = startingGains;
			}

			if (trialIteration % iterationsPerGain^2 == 0) {
				Yaw.KP += incrementBy;
				Yaw.KI = startingGains + incrementBy;
			}
			Yaw.KD += incrementBy;
			trialIteration += 1;
		}
	}
	else {
		sampleNumber++;
	}

}						  

int assignSerialInput(String serialInput) {

	if (serialInput[0] != '-') {
		Serial.println("ERROR: Expecting identifier. Ex: '-y 0.005' or '-A 150'");
		return -1;
	}

	char serialInputIdentifier = serialInput[1]; 
	double serialInputValue = serialInput.substring(3).toFloat();

	switch (serialInputIdentifier) {
		case('a') :
			Altitude.setpoint = serialInputValue; break;
		case('A') :
			Altitude.output = serialInputValue; break;
		case('y') :
			Yaw.setpoint = serialInputValue; break;
		case('Y') :
			Yaw.output = serialInputValue; break;
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

void lcdPrint(String value, int row, int column) {
	lcd.setCursor(column - 1, row - 1); // -1 so we can use natural numbers 
	lcd.print(value);
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

bool withinTolerance(double actual, double desired, double tolerancePercent) {
	return abs(desired - actual) <= tolerancePercent;
}

// instead of checking every sample should we average the samples and check against that?
bool eachSampleWithinTolerance(double samples[], int numberOfSamples, double desired, double tolerancePercent) {
	for (int i = 0; i < 100; i++) {
		if (!withinTolerance(samples[i], desired, tolerancePercent))
			return false;
	}
	return true;
}

bool trialHasTimedOut() {
	//todo:
	return false;
}

void encoderInteruptLineA() {


	if (digitalRead(YAW_INPUT_PIN_1) == HIGH) {

		if (digitalRead(YAW_INPUT_PIN_2) == LOW) 
			Yaw.rawInput = Yaw.rawInput + 1;         
		else 
			Yaw.rawInput = Yaw.rawInput - 1;         
	}
	else                                      
	{
		if (digitalRead(YAW_INPUT_PIN_2) == HIGH) 
			Yaw.rawInput = Yaw.rawInput + 1;          		
		else 
			Yaw.rawInput = Yaw.rawInput - 1;          
	}
}

void encoderInteruptLineB() {

	if (digitalRead(YAW_INPUT_PIN_2) == HIGH) {

		if (digitalRead(YAW_INPUT_PIN_1) == HIGH) 
			Yaw.rawInput = Yaw.rawInput + 1;        
		else 
			Yaw.rawInput = Yaw.rawInput - 1;        
	}
	else {

		if (digitalRead(YAW_INPUT_PIN_1) == LOW) 
			Yaw.rawInput = Yaw.rawInput + 1;          
		else 
			Yaw.rawInput = Yaw.rawInput - 1;         
	}

}