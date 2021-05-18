/*
Make a PPM stream from PWM's and Lighting connections on RC Controlled truck.
Copyright (C) 2021  Eddie Pounce

	Channel
		1		Proportional	5th Wheel Locking - to drive trailer legs (PWM from servo)
		2		Analogue		Rear/Stop Lights (uses trailer connector on controller)
		3		On/Off			Indicator	 (uses trailer connector on controller)
		4		On/Off			Indicator	 (uses trailer connector on controller)
		5		On/Off			Reversing Lighting (Opto Isolator in LED circuit)
		6		Proportional	2 way switch - to control Toggle Switch settings on Trailer
												(uses PWM output from reciever)


//#include <Arduino.h>
#include <EnableInterrupt.h>

void interruptReadChannels() {
	nowTime = micros();			// get current time - microseconds
	diffTime = nowTime - oldTime;	// calculate pulse length (time since last rising edge!)
	if (diffTime > minProportionalValue) {		// sort of debounce

setup	
enableInterrupt(inputPin, interruptReadChannels, PULSE_EDGE);

    Copyright

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

	84339861+eddiepounce@users.noreply.github.com

*/

#include <Arduino.h>

// -------------------------
//  Constants and variables to control functionality
//--------------------------
const int outPin 			= 12;	// pin for PPM stream
const int maxChannels		= 8;	// channels to create 
const int framePulseAndAddition = 500;  
				// length of pulses and half the additional time to make PPM give 1 to 2 mS
int debugMode = false;
//int debugMode = true;		// print frame values

//		1=legs, 2=rear/stop 3=ind, 4=ind, 5=reversing, 6=TSwitch
				
const char channelType[] =   {"-PASSSPSS"};  
//								12345678	// channel
//				P = Proportional (PWM) input
//				A = Analogue input to create 3 state channel (rear/stop lights)
//				//a = Analogue input to create 2 state channel 
//				S = Switch input - light on or off
//				T = Testing (analogue) - put input value in output array

//const int channelPIN[maxChannels+1] = {0,2,A3,4,5,A2,3,0,0};		// channel input pin
const int channelPIN[maxChannels+1] = {0,2,A3,4,5,6,3,0,0};		// channel input pin
//										 1  2 3 4 5 6 7 8  // channel
volatile static int	frameData[maxChannels+1] = {0,0,0,0,0,0,0,498,499};
//		0 - 1000 microS							  1 2 3 4 5 6 7 8  //channel
//volatile static unsigned long propInputTime[maxChannels+1] = {0,0,0,0,0,0,0,0,0};
volatile static unsigned long propInputTime1 = 0;
volatile static unsigned long propInputTime2 = 0;
//		used to store a time (micros) during input for proportional channels
// Video Camera Control
const int cameraControlPin = A0;
const int cameraControlChannel = 5;	// reversing light channel
unsigned long cameraRearOnTime = 0;
const int cameraRearOffWait = 1000;  // time to delay camera switch to front

// --------------------------------
// Proportional settings
// --------------------------------
const int propMinSetting = 0;		// min, mid and max pulse lengths
const int propMidSetting =500;		
const int propMaxSetting = 1000;
const int analogOffValue = 900;		// for Analogue input (inverted)
const int analogOnValue = 350;		// for Analogue input (inverted)
const int propMaxInput = 1200;		// maximum allowed proportional time (before error)	

// -------------------------
// for output of frames and debug info
//--------------------------
unsigned long frameTime = 0;
unsigned long nowTime = 0;
unsigned long monTime = 0;

// -------------------------
// Interrupt handler for channelx
//--------------------------
void interruptReadChannel1() {
	volatile static unsigned long tempTime;
	if (propInputTime1 > 0) {					// end of pulse
		tempTime = micros() - propInputTime1;	// pulse length
		if (tempTime < propMaxInput + 1000)  {	// if pulse a sensible length (we haven't got lost!)
			frameData[1] = tempTime - 1000;		// set frame data
			propInputTime1 = 0;					//   and zero the pulse time
		} else {
			propInputTime1 = micros();			// pulse too long - this must be the start!
		}
	} else {
		propInputTime1 = micros();				// start of pulse
	}
}
void interruptReadChannel2() {
	volatile static unsigned long tempTime;
	if (propInputTime2 > 0) {					// end of pulse
		tempTime = micros() - propInputTime2;	// pulse length
		if (tempTime < propMaxInput + 1000)  {	// if pulse a sensible length (we haven't got lost!)
			frameData[6] = tempTime - 1000;		// set frame data
			propInputTime2 = 0;					//   and zero the pulse time
		} else {
			propInputTime2 = micros();			// pulse too long - this must be the start!
		}
	} else {
		propInputTime2 = micros();				// start of pulse
	}
}
// -------------------------------
//		Setup
//--------------------------------
void setup() {
	Serial.begin(115200);
	if (debugMode) Serial.println("\n--- System Starting ---");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(outPin, OUTPUT);
	for (int i = 1; i <= maxChannels; i++){
		// set the input pins (hardware) as needed.
		if (channelPIN[i] > 0) pinMode(channelPIN[i], INPUT_PULLUP);
//		if (channelPIN[i] > 0 && i != 5) pinMode(channelPIN[i], INPUT_PULLUP);
//		if (channelPIN[i] > 0 && i == 5) pinMode(channelPIN[i], INPUT);
		
	}
	attachInterrupt(digitalPinToInterrupt(2), interruptReadChannel1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(3), interruptReadChannel2, CHANGE);
	// Camera control pin
	pinMode(cameraControlPin, OUTPUT);
	digitalWrite(cameraControlPin, LOW);	// default is front camera
}
	
// -------------------------------
//		Main Loop
//--------------------------------
void loop() {
	// read most of the channels 
	//		- proportional ones done via interrupts
	for (int i = 1; i <= maxChannels; i++){
		if (channelPIN[i] > 0) {
			if (channelType[i] == 'A') {
				int valueTemp = analogRead(channelPIN[i]);   // read the input pin
				if (valueTemp > analogOffValue) {
					frameData[i] = propMinSetting;
				} else {
					if (valueTemp < analogOnValue) {
						frameData[i] = propMaxSetting;
					} else {
						frameData[i] = propMidSetting;
					}
				}
			}
			if (channelType[i] == 'S') {
				if (digitalRead(channelPIN[i])) {		// read the input pin
					frameData[i] = propMaxSetting;
				} else {
					frameData[i] = propMinSetting;
				}
			}
//			if (channelType[i] == 'T') {
//				frameData[i] = analogRead(channelPIN[i]);
//				//frameData[i] = digitalRead(channelPIN[i]);
//			}
		}
	}
	
	
	// =========== output frame =============
	// every 20 milliSeconds
	nowTime = millis();
	if (nowTime - frameTime > 20) {
		for (int i = 1; i <= maxChannels; i++){
			digitalWrite(outPin, HIGH);		// ouput the channel pulse (500uS)
			delayMicroseconds(framePulseAndAddition);
			digitalWrite(outPin, LOW);
			delayMicroseconds(framePulseAndAddition+frameData[i]);
											// wait for the other 500uS and data time
		}
		digitalWrite(outPin, HIGH);			// ouput the end of frame pulse
		delayMicroseconds(framePulseAndAddition);
		digitalWrite(outPin, LOW);
		frameTime = nowTime;				// set frame output time
		digitalWrite(LED_BUILTIN, LOW);		// turn off LED

	// =========== Output Video Camera Control - Front/Rear =============
	// cameraControlChannel - controlled by reversing light on channel 5
	//				and waits for 1? seconds before switching back to the front camera.
		if (frameData[cameraControlChannel] > propMidSetting) {
			if (nowTime - cameraRearOnTime > cameraRearOffWait) {
				digitalWrite(cameraControlPin, LOW);
			}
		} else {
			digitalWrite(cameraControlPin, HIGH);
			cameraRearOnTime = nowTime;
		}
	

	}
			
	// ============== Output Debug info - Frame data and flash LED
	if ((millis() - monTime) > 1000) {  // every second
		digitalWrite(LED_BUILTIN, HIGH);	// turn on LED once a second
				
		if (debugMode) {
			Serial.print("\nFrame:  ");
			Serial.print("FrameTime: ");
			Serial.print(frameTime);
			Serial.print(";  ");
			for ( int i = 0; i <= maxChannels; i++ ){
				Serial.print("Ch");
				Serial.print(i);
				Serial.print(": ");
				Serial.print(frameData[i]);
				Serial.print("; ");
			}
			Serial.println("");
		}

		// Set debug on or off
		if (Serial.available() > 0) {
			if (Serial.readString() == "DebugON\n") {
				Serial.println("Debug is now turned on");
				debugMode = true;
			} else {
	//		if (Serial.readString() == "DebugOFF\n") 
				Serial.println("Debug is now turned off.  [reminder: DebugON]");
				debugMode = false;
			}
		}
	
		monTime = millis();
	}	
	
	
}

//   END		END			END			END












