/*
Make a PPM stream from PWM's and Lighting connections on RC Controlled truck.
Copyright (C) 2021  Eddie Pounce

Create pulse stream - 500us pulses with time between rising edges = RC PWM time (1-2ms).

	Channel
		1		Proportional	5th Wheel Lock Servo - to drive trailer legs (PWM input from MFU)
		2		Analogue		Rear/Stop Lights (uses trailer connector on MFU)
		3		On/Off			Indicator	 (uses trailer connector on MFU)
		4		On/Off			Indicator	 (uses trailer connector on MFU)
		5		On/Off			Reversing Lighting (via Opto Isolator in LED circuit)
		6		Proportional	3 way switch (centreSprung) - to control Toggle Switch settings on Trailer
												(PWM input from receiver)
		7		Proportional	3 way switch (3pos) - to control Gearbox  (PWM input from receiver)
												
					MFU = Multi Function Unit - Motor, 5th Wheel Lock, Lights & Sound Controller

	=============== MPU - mini processor unit - Arduino Nano ===================
	Gnd	Black			# IR Emitter
	D2			Ch1		Copy of 5th Wheel Lock Servo - to drive trailer legs (PWM input)
	D3	Green	Ch6		Rx5 - 3 way switch - to control Toggle Switch settings on Trailer  (10k resistor)
												(PWM input from receiver)
	D4			Ch3		Indicator (uses trailer connector on MFU)
	D5			Ch4		Indicator (uses trailer connector on MFU)
	D6	Red		Ch5		Reversing Lighting (via Opto Isolator in LED circuit)
	D7	Green			Camera Control - Selects forward or reverse camera				#########  Moved from A0
	D8	Yellow			Throttle monitoring (PWM input)
	D9	Purple	PWM 	Cab Lighting	
	D10	Light Grey		Gearbox Servo from MFU  (28awg ribbon #2)
	D11	Dark Purple		Feed to Gearbox Servo	(28awg ribbon #3)
	D12	White			# IR Emitter
	A0	White (10k resistor)	From Rx6 - 3 pos switch.  (28awg ribbon #1)  [Frame position 7]
	A1	Black			RightIndRepeater control
	A2	Red				LeftIndRepeater control
	A3	Yellow	Ch2		Rear/Stop Lights (uses trailer connector on MFU) 
												(converts to High/Mid/Low)
	A4	White			SideLights control
	A5	Orange			Power switch control for Camera's 
	5V	Red		Power	(from MFU from Copy of 5th Wheel Lock Servo)
	Gnd	Black	Power	(from MFU from Copy of 5th Wheel Lock Servo)
	=================
	
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
// Channels
//		1=legs, 2=rear/stop 3=ind, 4=ind, 5=reversing, 6=TSwitch
//		Throttle monitor.
				
const char channelType[] =   {"-PASSSPPS"};  
//								12345678	// channel
//				P = Proportional (PWM) input
//				A = Analogue input to create 3 state channel (rear/stop lights)
//				//a = Analogue input to create 2 state channel 
//				S = Switch input - light on or off
//				T = Testing (analogue) - put input value in output array

const int channelPIN[maxChannels+1] = {0,2,A3,4,5,6,3,A0,0};		// channel input pin
//										 1  2 3 4 5 6  7 8  // channel
volatile static int	frameData[maxChannels+1] = {0,0,0,0,0,0,0,498,499};
//		0 - 1000 microS							  1 2 3 4 5 6 7 8  //channel
//volatile static unsigned long propInputTime[maxChannels+1] = {0,0,0,0,0,0,0,0,0};
volatile static unsigned long propInputTime1 = 0;
volatile static unsigned long propInputTime6 = 0;
//		used to store a time (micros) during input for proportional channels (ext interrupts)

// Marker Lights & Indicator Repeaters
const int ctrlRightIndRepeater = A1;
const int ctrlLeftIndRepeater = A2;
const int ctrlSideLights = A4;

// Video Camera Control / Trottle monitoring
const int cameraControlPin = 7;			// Selects forward or reverse camera
const int cameraPowerPin = A5;				// Power control for cameras
const int throttlePin = 8;					// Throttle monitoring pin
const int throttleReverseValue = 550;		// if more - set motion to Reverse  // for camera
const int throttleForwardValue = 450;		// if less - set motion to Forward  // for camera
volatile static unsigned long throttleStartTime = 0;
//		used to store a time (micros) during input for pin change interrupt
volatile static int throttleValue = 500;	// initial value - mid point
volatile static bool throttleState = LOW;
bool braked = false;			// have we braked after going forward?
bool reversing = false;			// are we now reversing?
const int ctrlCabLightingPin = 9;			// turns cab lighting on/off
const int gearboxFromMFUPin = 10;			// PWM from MFU to feed to gearbox servo
const int gearboxServoPin = 11;				// PWM to Gearbox Servo
int gearboxGear = 2;
int gearboxGearOld = 0;
int gearboxPulseCount = 20;						// controls output to servo and number of pulses to send
volatile static unsigned long gearboxStartTime = 0;
//		used to store a time (micros) during input for pin change interrupt
volatile static int gearboxFromMFUValue = 500;			// initial value - mid point
volatile static bool gearboxFromMFUState = LOW;		// store last value for timing input

volatile static unsigned long switchStartTime = 0;	// channelPIN[7] = A0
//		used to store a time (micros) during input for pin change interrupt
bool gearboxControlToggle = HIGH;			//LOW == stick; HIGH == switch;
unsigned long tSwitchDownTimerStart = 0;
unsigned long tSwitchTimerNow = 0;


// --------------------------------
// Proportional settings
// --------------------------------
const int propMinSetting = 0;		// min, mid and max pulse lengths
const int propOffSetting = 300;
const int propMidSetting =500;		
const int propOnSetting = 700;
const int propMaxSetting = 1000;
const int analogOffValue = 900;		// for Analogue input (inverted)
const int analogOnValue = 350;		// for Analogue input (inverted)


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
	if(digitalRead(channelPIN[1])) {
		propInputTime1 = micros();		
	} else {
		frameData[1] = micros() - propInputTime1 - 1000;	// 
	}
}

void interruptReadChannel6() {
	if(digitalRead(channelPIN[6])) {
		propInputTime6 = micros();		
	} else {
		frameData[6] = micros() - propInputTime6 - 1000;	// 
	}
}

void pciSetup(byte pin) {
	// Install Pin Change Interrupt (PCI) for a pin (can be called multiple times)
	//			(care needed in ISR if multiple pins on a port are used)
   *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}	
	//Pin Change Interrupt Request 0 (pins D8 to D13) port B (PCINT0_vect)
	//Pin Change Interrupt Request 1 (pins A0 to A5) port C  (PCINT1_vect)
	//Pin Change Interrupt Request 2 (pins D0 to D7) port D  (PCINT2_vect)
	//PCICR |= 0b00000001;    // turn on port B (PCINT0 – PCINT7)
	//PCICR |= 0b00000010;    // turn on port C (PCINT8 – PCINT14)
	//PCICR |= 0b00000100;    // turn on port D (PCINT16 – PCINT23)


ISR (PCINT0_vect) {
    // For PCINT of pins D8 to D13 - Port B		(8 & 10 used)				//=======================================
	//  if (PINB & B00000001)  Pin8		throttlePin
	//  if (PINB & B00000100)  Pin10	gearboxFromMFUPin
	
	if(!throttleState) {				// current state low
		if (PINB & B00000001) {			// Pin high & old state low - thats us
			throttleState = HIGH;
			throttleStartTime = micros();
		}
	} else {							// current state high
		if (!(PINB & B00000001)) {		// Pin low & old state high - thats us
			throttleState = LOW;
			throttleValue = micros() - throttleStartTime - 1000;
		}
	} 
	if(!gearboxFromMFUState) {			// current state low
		if (PINB & B00000100) {			// Pin high & old state low - thats us
			gearboxFromMFUState = HIGH;
			gearboxStartTime = micros();		
		}
	} else {							// current state high
		if (!(PINB & B00000100)) {		// Pin low & old state high - thats us
			gearboxFromMFUState = LOW;
			gearboxFromMFUValue = micros() - gearboxStartTime - 1000;
		}
	} 	
}
 
ISR (PCINT1_vect) {
    // For PCINT of pins A0 to A5 - Port C		(A0 used)
	// Only 1 pin used in port
//	if(digitalRead(channelPIN[7])) {
	if(PINC & B00000001) {
		switchStartTime = micros();		
	} else {
		frameData[7] = micros() - switchStartTime - 1000;	// 
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
	}
	attachInterrupt(digitalPinToInterrupt(channelPIN[1]), interruptReadChannel1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(channelPIN[6]), interruptReadChannel6, CHANGE);
	// Camera power & control pins
	pinMode(cameraPowerPin, OUTPUT);
	pinMode(cameraControlPin, OUTPUT);
	digitalWrite(cameraPowerPin, HIGH);		// turn on Camera power
	digitalWrite(cameraControlPin, LOW);	// default is front Camera
	pinMode(throttlePin, INPUT);
		// enable interrupt for pin...  -- Pin Change Interrupt (PCI)
	pciSetup(throttlePin);									// ##########################################
		//PCICR  |= B00000001;			//"PCIE0" enabeled (PCINT0 to PCINT7)
		//PCMSK0 |= B00000001;			//"PCINT0" enabeled -> D8 will trigger interrupt
	// setup for switch on Rx6
	//pinMode(channelPIN[7], INPUT); - done in loop
		// enable interrupt for pin...  -- Pin Change Interrupt (PCI)
	pciSetup(channelPIN[7]);
	// Gearbox Control
	pciSetup(gearboxFromMFUPin);
	pinMode(gearboxFromMFUPin, INPUT);
	pinMode(gearboxServoPin, OUTPUT);
	// Cab lighting control pin
	pinMode(ctrlCabLightingPin, OUTPUT);
	digitalWrite(ctrlCabLightingPin, HIGH);		// cab lights on
	// Marker Lights & Indicator Repeaters
	pinMode(ctrlSideLights, OUTPUT);
	digitalWrite(ctrlSideLights, LOW);
	pinMode(ctrlLeftIndRepeater, OUTPUT);
	digitalWrite(ctrlLeftIndRepeater, LOW);
	pinMode(ctrlRightIndRepeater, OUTPUT);
	digitalWrite(ctrlRightIndRepeater, LOW);
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
	
	// =========== Output Frame =============
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

		// =========== Gearbox Control =============	
			// decide if stick or switch being used for Gearbox control
			// channel 6 switch - hold down (high) for >2 seconds to toggle
			// gearboxControlToggle = LOW == stick; HIGH == switch;
		
		tSwitchTimerNow = millis();
		if (tSwitchDownTimerStart == 0 && frameData[6] <= propOffSetting) {		// start down timmer
			tSwitchDownTimerStart = tSwitchTimerNow;
		} 
		if (frameData[6] > propOffSetting && frameData[6] < propOnSetting) {	// force mid point for switch
				frameData[6] = propMidSetting;
			} 
		if (tSwitchDownTimerStart > 0 && frameData[6] == propMidSetting) {
			if (tSwitchTimerNow - tSwitchDownTimerStart > 2000) {				// switch was held for more than 2 seconds
				gearboxControlToggle = !gearboxControlToggle;					// toggle the gearbox control method
			}
			tSwitchDownTimerStart = 0;					// reset down timmer
		}

		if (gearboxControlToggle) {					//true - gearboxControlToggle = HIGH = using switch
			if (frameData[7] < propOffSetting) {			// Code if switch being used
				gearboxGear = 1;
			} else if (frameData[7] > propOnSetting) {
				gearboxGear = 3;
			} else {
				gearboxGear = 2;
			}
		} else {
			if (gearboxFromMFUValue < propOffSetting) {		// Code if stick being used
				gearboxGear = 3;
			} else if (gearboxFromMFUValue > propOnSetting) {
				gearboxGear = 1;
			} else {
				gearboxGear = 2;
			}
		}
		if (gearboxGear != gearboxGearOld) {
			gearboxPulseCount = 0;
			gearboxGearOld = gearboxGear;
		}
		// Put into new gear if throttle open or if going to 2nd gear
		if (gearboxPulseCount < 10) {
//			if (throttleValue > throttleReverseValue || throttleValue < throttleForwardValue || gearboxGear == 2) {
				// because motor doesn't turn till quite a way off neutral, throttleValue 's are as follows
			if ((throttleValue > 750 && reversing) || (throttleValue < 300) || gearboxGear == 2) {
				gearboxPulseCount += 1;
				digitalWrite(gearboxServoPin, HIGH); 
				delayMicroseconds(500 + (gearboxGear * 500));
				digitalWrite(gearboxServoPin, LOW); 
			}
		}
		
		

		// =========== Output Video Camera Control - Front/Rear =============
			// cameraControlPin - controlled by monitoring Throttle
		if (throttleValue > throttleReverseValue) {
			if (reversing) {
				digitalWrite(cameraControlPin, HIGH);		// rear camera
				digitalWrite(ctrlCabLightingPin, LOW);		// moving so cab light off
				digitalWrite(cameraPowerPin, HIGH);			// and cameras on
				if (debugMode) {
					Serial.print(" Reversing. Throttle value: ");
					Serial.println(throttleValue);
				}
			} else {
				braked = true;			// camera doesn't change if just braking.
			}
		}
		if (throttleValue < throttleReverseValue and braked == true) {
			reversing = true;
		}
		if (throttleValue < throttleForwardValue) {
			digitalWrite(cameraControlPin, LOW);	// front camera
			digitalWrite(ctrlCabLightingPin, LOW);			// moving so cab light off
			digitalWrite(cameraPowerPin, HIGH);			// and cameras on
			braked = false;
			reversing = false;
		}
//		if (debugMode) {
//			Serial.print(" Braked: ");
//			Serial.println(braked);
//		}

		// turn cameras off if stationary for xxx time
			//digitalWrite(cameraPowerPin, LOW);
		
		// =========== Side Lights Control =============
		if (frameData[2] == propMidSetting) digitalWrite(ctrlSideLights, HIGH);	
			// Don't use "else" or marker lights go out when stop lights are on.
		if (frameData[2] == propMinSetting) digitalWrite(ctrlSideLights, LOW);	
		// =========== Indicator Repeaters (in steps) Control =============
			// channel 3 & 4 = Indicators, MinSetting is ON, 
		if (frameData[3] == propMinSetting) {
			digitalWrite(ctrlLeftIndRepeater, HIGH);	
		} else {
			digitalWrite(ctrlLeftIndRepeater, LOW);	
		}
		if (frameData[4] == propMinSetting) {
			digitalWrite(ctrlRightIndRepeater, HIGH);	
		} else {
			digitalWrite(ctrlRightIndRepeater, LOW);	
		}
		// =========== Cab Lighting Control =============
			// cameraControlPin - controlled by monitoring Throttle
			// channel 3 & 4 = Indicators, MinSetting is ON, So when Hazards on:
		if (frameData[3] == propMinSetting && frameData[4] == propMinSetting) 
							digitalWrite(ctrlCabLightingPin, HIGH); // Cab Lights on
			// turned off in camera control section.
	}

	// ============== Output Debug info - Frame data and flash LED
	if ((millis() - monTime) > 1000) {  // every second
		digitalWrite(LED_BUILTIN, HIGH);	// turn on LED once a second
		// CAB Light:  On at startup, Off once trottle moved
		
				
		if (debugMode) {
			Serial.print("Frame:  ");
			Serial.print("Time: ");
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

			Serial.print("Throttle value: ");
			Serial.print(throttleValue);
			Serial.print(". GearboxControl: Toggle: ");
			Serial.print(gearboxControlToggle);
			Serial.print(". Gear: ");
			Serial.print(gearboxGear);
			Serial.print(". MFUVal: ");
			Serial.print(gearboxFromMFUValue);
			Serial.println("");
		}

		// Set debug on or off
		if (Serial.available() > 0) {
			String monSerialRead = Serial.readString();
			monSerialRead.trim();               // remove any \r \n whitespace at the end of the String

			if (monSerialRead == "cabon") digitalWrite(ctrlCabLightingPin, HIGH);
			if (monSerialRead == "caboff") digitalWrite(ctrlCabLightingPin, LOW);

			if (monSerialRead == "1") {gearboxGear = 1; gearboxPulseCount = 0;}
			if (monSerialRead == "2") {gearboxGear = 2; gearboxPulseCount = 0;}
			if (monSerialRead == "3") {gearboxGear = 3; gearboxPulseCount = 0;}
						
			if (monSerialRead == "DebugON" || monSerialRead == "d") {
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

