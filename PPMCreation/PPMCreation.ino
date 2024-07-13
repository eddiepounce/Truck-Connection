/*
Make a PPM stream from PWM's and Lighting connections on RC Controlled truck.
Copyright (C) 2021  Eddie Pounce

Create pulse stream - 500us pulses with time between rising edges = RC PWM time (1-2ms).

	Channel		Inputs
		1		Proportional	5th Wheel Lock Servo - for trailer legs (PWM from MFU)
		2		Analogue		Rear/Stop Lights(uses trailer connector on MFU)
		3		Analogue		Indicator	 	(uses trailer connector on MFU)
		4		Analogue		Indicator	 	(uses trailer connector on MFU)
		5		On/Off			Reversing Lighting (via Throttle monitoring - sticky ON in Reverse)
		6		Proportional	3 way sw (centre) - for Toggle Switch on Trailer
									and for Gearbox control method	(PWM from receiver)
		7		Proportional	3 way sw (3pos) - for Gearbox		(PWM from receiver)
	Internal use inputs
		-		Proportional	Throttle Monitor		(PWM from receiver)
		-		Proportional	Gearbox Servo			(PWM from MFU)
				
	MFU = Multi Function Unit - Motor, 5th Wheel Lock, Lights & Sound Controller

	=============== MPU - mini processor unit - Arduino Nano ===================
	Gnd	Black			# IR Emitter (Trailer feed)
I	D2	Yellow	Ch1	10k	Copy of 5th Wheel Lock Servo - for trailer legs (10k resistor)
																(PWM from MFU)
I	D3	Green	Ch6	10k	Rx5 - 3 way switch - for Toggle Switch settings on Trailer  (10k resistor)
								and for Gearbox control method	(PWM from receiver)
O	D4	White			# IR Emitter (Trailer feed)
I	D5	Red		Ch5		Input NOT USED - Reversing Light via Opto Isolator in LED circuit(pull-up needed)
O	D6	Blue			--reserved for gear display	(28awg ribbon #a)
O	D7	Green			--reserved for gear display	(28awg ribbon #b)
O	D8	Purple	PWM? 	Cab Lighting Control (takes 0.43mA)
I	D9	Yellow			Throttle monitoring (PWM from receiver)  (chip output may be broken / may be not)
I	D10	Light Grey		Gearbox Servo (PWM from MFU)	(28awg ribbon #2)
O	D11	Dark Purple		Feed to Gearbox Servo			(28awg ribbon #3)
O	D12	Green			Camera Control - Selects forward or reverse camera
I	A0	White 	Ch7	10k	Rx6 - 3 pos switch				(28awg ribbon #1)  (10k resistor) 
																(PWM from receiver)
O	A1	Black			RightIndRepeater control
O	A2	Red				LeftIndRepeater control
I	A3	Blue	Ch2		Rear/Stop Lights input (uses trailer connector on MFU)(pull-up needed)
											(Analog - converts to High/Mid/Low)
O	A4	White			SideLights control
O	A5	Orange			Power switch control for Camera's  (** Not really Needed **)
I	A6	Yellow	Ch3		Indicator input (uses trailer connector on MFU) (pull-up needed:hardwired)
l	(Analog Input Ony)						(Analog - converts to High/Mid/Low)
I	A7	Red		Ch4		Indicator input (uses trailer connector on MFU)	(pull-up needed:hardwired)
	(Analog Input Only)						(Analog - converts to High/Mid/Low)
	
	5V	Red		Power	Copy of 5th Wheel Lock Servo)	(power from MFU)
	Gnd	Black	Power	Copy of 5th Wheel Lock Servo)	(power from MFU)

	=================
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
const int outPin 			= 4;	// pin for PPM stream
const int maxChannels		= 8;	// channels to create 
const int framePulseAndAddition = 500;  
				// length of pulses and half the additional time to make PPM give 1 to 2 mS
int debugMode = false;
//int debugMode = true;		// print frame values

// Channels
//		1=legs, 2=rear/stop 3=ind, 4=ind, 5=reversing, 6=TSwitch, 7=TrailerRamps
				
const char channelType[] =   {"-PAAASPPS"};  
//								12345678	// channel
//				P = Proportional (PWM) input
//				A = Analogue input to create 3 state channel (e.g rear/stop lights)
//				S = Digital input - on or off

const int channelPIN[maxChannels+1] = {0,2,A3,A6,A7,0,3,A0,0};	// channel input pin
//										 1  2  3  4 5 6  7 8	// channel
const char channelPINpullup[] =   {"--P--P---"};// P = internal pull-up needed
//									-12345678  // channel
volatile static int	frameData[maxChannels+1] = {500,500,500,500,500,500,500,498,499};
//		0 - 1000 microS								  1   2   3   4    5   6   7   8  //channel
volatile static unsigned long propInputTime1 = 0;
volatile static unsigned long propInputTime6 = 0;
//		used to store a time (micros) during input for proportional channels (ext interrupts)

// Marker Lights & Indicator Repeaters
const int ctrlRightIndRepeater = A1;
const int ctrlLeftIndRepeater = A2;
const int ctrlSideLights = A4;

// Video Camera Control / Trottle monitoring
const int cameraControlPin = 12;			// Selects forward or reverse camera
const int cameraPowerPin = A5;				// Power control for cameras
const int throttlePin = 9;					// Throttle monitoring pin
const int throttleReverseValue = 550;		// if more - set motion to Reverse  // for camera
const int throttleForwardValue = 450;		// if less - set motion to Forward  // for camera
volatile static unsigned long throttleStartTime = 0;
//		used to store a time (micros) during input for pin change interrupt
volatile static int throttleValue = 500;	// initial value - mid point
volatile static bool throttleState = LOW;
bool braked = false;			// have we braked after going forward?
bool reversing = false;			// are we now reversing?
const int ctrlCabLightingPin = 8;			// turns cab lighting on/off
const int gearboxFromMFUPin = 10;			// PWM from MFU to feed to gearbox servo
const int gearboxServoPin = 11;				// PWM to Gearbox Servo
int gearboxGear = 2;
int gearboxGearOld = 1;             // Don't set at 0; valid values - 1 - 1st, 2 = 2nd, 3 = 3rd.
int gearboxPulseCount = 100;		// controls output to servo and number of pulses to send (high value so no initialisation)
volatile static unsigned long gearboxStartTime = 0;
//		used to store a time (micros) during input for pin change interrupt
volatile static int gearboxFromMFUValue = 500;			// initial value - mid point
volatile static bool gearboxFromMFUState = LOW;		// store last value for timing input
int gearboxInDelay = 0;		// For debug of gearbox timing input from keyboard.

volatile static unsigned long switchStartTime = 0;	// channelPIN[7] = A0
//		used to store a time (micros) during input for pin change interrupt
bool gearboxControlToggle = LOW;       //  Start using Stick -- LOW == stick; HIGH == switch;
bool gearboxShiftStarted = false;		// Make sure only 1 gear change per stick movement.
unsigned long tSwitchDownTimerStart = 0;
unsigned long tSwitchTimerNow = 0;
// Outputs to show what gear we are in
const int gearboxShowGear1 = 6;
const int gearboxShowGear2 = 7;



// --------------------------------
// Proportional settings
// --------------------------------
const int propMinSetting = 0;		// min, mid and max pulse lengths
const int propOffSetting = 300;
const int propMidSetting =500;		
const int propOnSetting = 700;
const int propMaxSetting = 1000;
//const int analogOffValue = 900;		// for Analogue input (inverted)
//const int analogOnValue = 300;		// for Analogue input (inverted)
const int analogOffValue = 300;		// for Analogue input
const int analogOnValue = 900;		// for Analogue input


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
    // For PCINT of pins D8 to D13 - Port B		(8 & 10 used)	
	//  if (PINB & B00000001)  Pin8		throttlePin
	//  if (PINB & B00000100)  Pin10	gearboxFromMFUPin
	
	if(!throttleState) {				// current state low
//		if (PINB & B00000001) {			// Pin high & old state low - thats us
		if (PINB & B00000010) {			// Pin high & old state low - thats us
			throttleState = HIGH;
			throttleStartTime = micros();
		}
	} else {							// current state high
//		if (!(PINB & B00000001)) {		// Pin low & old state high - thats us
		if (!(PINB & B00000010)) {		// Pin low & old state high - thats us
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
		if (channelPIN[i] > 0) pinMode(channelPIN[i], INPUT);
	}
	pinMode(A3, INPUT_PULLUP);
	pinMode(5, INPUT_PULLUP);
	//  D5 - from Opto Isolator in LED circuit needs INPUT_PULLUP But NOT used
	//  A3, A6, A7 - from the MFU Trailer Feed need INPUT_PULLUP 
	//          (A6 & A7 Hardwired as INPUT_PULLUP does not seem to work on these 2 pins!!)
		
	attachInterrupt(digitalPinToInterrupt(channelPIN[1]), interruptReadChannel1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(channelPIN[6]), interruptReadChannel6, CHANGE);
	//pinMode(channelPIN[7], INPUT); - done in loop
	pciSetup(channelPIN[7]);  
		// enable interrupt for pin...  -- Pin Change Interrupt (PCI)

  // Throttle Monitoring
	pinMode(throttlePin, INPUT);
	pciSetup(throttlePin);
		// enable interrupt for pin...  -- Pin Change Interrupt (PCI)
		//PCICR  |= B00000001;			//"PCIE0" enabeled (PCINT0 to PCINT7)
		//PCMSK0 |= B00000001;			//"PCINT0" enabeled -> D8 will trigger interrupt

  // Gearbox Control
	pinMode(gearboxFromMFUPin, INPUT);
	pciSetup(gearboxFromMFUPin);
		// enable interrupt for pin...  -- Pin Change Interrupt (PCI)
	pinMode(gearboxServoPin, OUTPUT);
	pinMode(gearboxShowGear1, OUTPUT);
	pinMode(gearboxShowGear1, OUTPUT);

  // Camera power & control pins
	pinMode(cameraPowerPin, OUTPUT);
	pinMode(cameraControlPin, OUTPUT);
	digitalWrite(cameraPowerPin, HIGH);		// turn on Camera power
	digitalWrite(cameraControlPin, LOW);	// default is front Camera
	// setup for switch on Rx6
 
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
	
	//Serial.println("Setup done");

	
}
	
// -------------------------------
//		Main Loop
//--------------------------------
void loop() {
	// read non-proportional channels - Proportional done via interrupts
	for (int i = 1; i <= maxChannels; i++){
		if (channelPIN[i] > 0) {
			if (channelType[i] == 'A') {
				int valueTemp = analogRead(channelPIN[i]);   // read the input pin
				if (valueTemp < analogOffValue) {
					frameData[i] = propMaxSetting;
				} else {
					if (valueTemp > analogOnValue) {
						frameData[i] = propMinSetting;
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
		}
	}
	
	// =========== Output Frame =============
	// every 20 milliSeconds
	nowTime = millis();
	if (nowTime - frameTime > 20) {			// Pulses are high, data is in low time
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
		
		if (frameData[6] > propOffSetting && frameData[6] < propOnSetting) { // if switch not high or low
				frameData[6] = propMidSetting;								 //     set to mid point
		} 										// -- a bit redundant as this is a switch not proportional
		tSwitchTimerNow = millis();
		if (tSwitchDownTimerStart == 0 && frameData[6] <= propOffSetting) {	// start down timer
			tSwitchDownTimerStart = tSwitchTimerNow;
		} 
		if (tSwitchDownTimerStart > 0 && frameData[6] == propMidSetting) {	
															// timer underway & switch back at middle
			if (tSwitchTimerNow - tSwitchDownTimerStart > 2000) { // switch was held for more than 2s
				gearboxControlToggle = !gearboxControlToggle;	  // toggle the gearbox control method
			}
			tSwitchDownTimerStart = 0;					// reset down timer because switch back at middle
		}

		if (gearboxControlToggle) {					//true - gearboxControlToggle = high = using switch
                // ** Switch ** being used	// Switch = up = 1st = servo ant-clock = gear control forward.
											// Switch = down = 3rd = servo clockwise = gear control backward.
                                            //      		up = 983. 496, down = -12
			if (frameData[7] < propOffSetting) {
				gearboxGear = 3;
			} else if (frameData[7] > propOnSetting) {
				gearboxGear = 1;
			} else {
				gearboxGear = 2;
			}
		} else {									// *** Stick ***
				//  Changing way stick works - use as up/down mover instead of being in direct control
				// 		(so your don't have to hold the stick over for 1st & 3rd)
                // ** Stick ** being used	// Stick = left = down = servo ant-clock = gear control forward.
											// Stick = right = up = servo clockwise = gear control backward.
											//     			 left = 84, 532, right = 984
				// bool gearboxShiftStarted = false; - so only 1 change per stick move
			if (!gearboxShiftStarted && gearboxFromMFUValue < propOffSetting) {			// down
				gearboxShiftStarted = true;
				gearboxGear -= 1;
			} else if (!gearboxShiftStarted && gearboxFromMFUValue > propOnSetting) {	// up
				gearboxShiftStarted = true;
				gearboxGear += 1;
			} else if (gearboxFromMFUValue > propOffSetting && gearboxFromMFUValue < propOnSetting){
				gearboxShiftStarted = false;
			}
		}
		
		if (gearboxGear != gearboxGearOld) {			// gear being changed
			if (gearboxGear < 1) gearboxGear = 1;		// allow retry to get into gear
			if (gearboxGear > 3) gearboxGear = 3;		// allow retry to get into gear
			gearboxPulseCount = 0;
			gearboxGearOld = gearboxGear;			
		}

		if (gearboxPulseCount < 25) {			// 17 or 25 frames used to set servo position
			gearboxPulseCount += 1;
			digitalWrite(gearboxServoPin, HIGH); 

			if (gearboxInDelay == 0) {		// For debug of gearbox timing from keyboard!!
				const int gearboxGearServoValue[4] = {1500, 1900, 1505, 1000};
				// delayMicroseconds(500 + (gearboxGear * 500));  // Correct but wrong way round.
				delayMicroseconds(gearboxGearServoValue[gearboxGear]);
			} else {
				delayMicroseconds(gearboxInDelay);   // For debug of gearbox timing.
			}
			digitalWrite(gearboxServoPin, LOW); 
		} else {
			gearboxInDelay = 0;
		}

		// =========== Throttle Monitoring - to set Reverse mode =============
		// =========== Video Camera Control - Front/Rear
		// =========== Cab Light - On/Off
		// =========== frameData[5] also set here to make reverse on trailer stay on when stationary
		//							(pin D5 (reversing) input not used to set frameData[5]).
		if (throttleValue > throttleReverseValue) {		// Brake/Reverse
			if (reversing) {
				frameData[5] = propMaxSetting;				// trailer reversing light on
				digitalWrite(cameraPowerPin, HIGH);			// cameras on
				digitalWrite(cameraControlPin, HIGH);		// rear camera
				digitalWrite(ctrlCabLightingPin, LOW);		// moving so cab light off
				if (debugMode && ((millis() - monTime) > 1000)) {
					Serial.print(" Reversing. Throttle value: ");
					Serial.println(throttleValue);
				}
			} else {
				braked = true;				// camera doesn't change if just braking.
			}
		}
		if (throttleValue < throttleReverseValue and braked == true) {
			reversing = true;
		}
		if (throttleValue < throttleForwardValue) {		// Forward
			frameData[5] = propMinSetting;					// trailer reversing light off
			digitalWrite(cameraPowerPin, HIGH);				// camera on
			digitalWrite(cameraControlPin, LOW);			// front camera
			digitalWrite(ctrlCabLightingPin, LOW);			// moving so cab light off
			braked = false;
			reversing = false;
		}
//		if (debugMode) {Serial.print(" Braked: ");Serial.println(braked);}
// turn cameras off if stationary for xxx time
					//digitalWrite(cameraPowerPin, LOW);
		
		// =========== Side Lights Control =============
		if (frameData[2] == propMidSetting) digitalWrite(ctrlSideLights, HIGH);	
			// Don't use "else" or marker lights go out when stop lights are on.
		if (frameData[2] == propMinSetting) digitalWrite(ctrlSideLights, LOW);	
		// =========== Indicator Repeaters Control (in steps of Truck) =============
			// channel 3 & 4 = Indicators, MinSetting is ON, 
		if (frameData[3] == propMaxSetting) {
			digitalWrite(ctrlLeftIndRepeater, HIGH);	
		} else {
			digitalWrite(ctrlLeftIndRepeater, LOW);	
		}
		if (frameData[4] == propMaxSetting) {
			digitalWrite(ctrlRightIndRepeater, HIGH);	
		} else {
			digitalWrite(ctrlRightIndRepeater, LOW);	
		}
		// =========== Cab Lighting Control =============
			// cameraControlPin - controlled by monitoring Throttle
			// channel 3 & 4 = Indicators, MinSetting is ON, So when Hazards on:
		if (frameData[3] == propMaxSetting && frameData[4] == propMaxSetting) 
							digitalWrite(ctrlCabLightingPin, HIGH); // Cab Lights on
			// turned off in camera control section.
	}

	// ============== Output Debug info - Frame data and flash LED
	if ((millis() - monTime) > 1000) {  // every second
		digitalWrite(LED_BUILTIN, HIGH);	// turn on LED once a second
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
			Serial.println("");	// new line
			Serial.print("Throttle value: ");
			Serial.print(throttleValue);
			Serial.print(". GearboxControl: Toggle: ");
			Serial.print(gearboxControlToggle);
			Serial.print(". Gear: ");
			Serial.print(gearboxGear);
			Serial.print(". StickVal: ");
			Serial.print(gearboxFromMFUValue);
			Serial.print(". Reversing: ");
			Serial.print(reversing);
			Serial.println("");
		}

		// Set debug on or off
		if (Serial.available() > 0) {
			String monSerialRead = Serial.readString();
			monSerialRead.trim();               // remove any \r \n whitespace at the end of the String

			if (monSerialRead == "h") digitalWrite(gearboxShowGear1, HIGH);
			if (monSerialRead == "l") digitalWrite(gearboxShowGear1, LOW);
			if (monSerialRead == "h") digitalWrite(gearboxShowGear2, HIGH);
			if (monSerialRead == "l") digitalWrite(gearboxShowGear2, LOW);
			if (monSerialRead == "h") digitalWrite(ctrlCabLightingPin, HIGH);
			if (monSerialRead == "l") digitalWrite(ctrlCabLightingPin, LOW);
			if (monSerialRead == "h") digitalWrite(cameraControlPin, HIGH);
			if (monSerialRead == "l") digitalWrite(cameraControlPin, LOW);
			if (monSerialRead == "h") Serial.println("Set of pins HIGH.");
			if (monSerialRead == "l") Serial.println("Set of pins LOW.");

			if (monSerialRead == "cabon") {digitalWrite(ctrlCabLightingPin, HIGH); Serial.println("Cab on.");}
			if (monSerialRead == "caboff") {digitalWrite(ctrlCabLightingPin, LOW); Serial.println("Cab off.");}

			if (monSerialRead == "g1") {gearboxGear = 1; gearboxPulseCount = 0; Serial.println("1st Gear.");}
			if (monSerialRead == "g2") {gearboxGear = 2; gearboxPulseCount = 0; Serial.println("2nd Gear.");}
			if (monSerialRead == "g3") {gearboxGear = 3; gearboxPulseCount = 0; Serial.println("3rd Gear.");}
						
			if (monSerialRead == "DebugON" || monSerialRead == "d") {
				Serial.println("Debug is now turned on.  [reminder: DebugOFF / off]");
				Serial.println("  Other options:  h/l  and  cabon/caboff");
				debugMode = true;
			}	
			if (monSerialRead == "DebugOFF" || monSerialRead == "off") {
				Serial.println("Debug is now turned off.  [reminder: DebugON / d]");
				debugMode = false;
			}
			gearboxInDelay = monSerialRead.toInt();
			if (gearboxInDelay != 0) {
				gearboxPulseCount = 0;
				Serial.print("gearboxInDelay = ");
				Serial.println(gearboxInDelay);
			}
		}
		monTime = millis();
	}		
}

//   END		END			END			END

