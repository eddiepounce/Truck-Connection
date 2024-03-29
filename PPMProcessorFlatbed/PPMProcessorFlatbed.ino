/*
Input is PPM - usually from IR Receiver at Trailer 5th Wheel King Pin.
Outputs individual channels on a pin.
Copyright (C) 2021  Eddie Pounce

Read pulse stream created by tractor unit:
	- 500us pulses with time between rising edges = RC PWM time (1-2ms).


NOTE: ----- The Nano only has PWM on Pins  3,5,6,9,10,11. -------- 
					(not 13 !!!)

LED Names: LED_BUILTIN  Pin 13

Still To Do:
		Failsafe

==============================================================

Lorry Trailer (EDP1) - 40ft Flat

Channel		Function				Pin		Info								Implementation
-------		--------				---		----								--------------
	1 	 	Legs					A4		Prop to ESC to control Legs Motor	P - Proportional
	2 	 	Rear/Stop Lights		6		PWM to LEDs (Off/Mid/High)			l - Prop whole swing - LED driver
	3 	 	Indicator Left			8		Off/On								S - using direction - LED driver
	4 	 	Indicator Right			9		Off/On								S - using direction - LED driver
	5 	 	Reversing Light			11		Off/On								S - using direction - LED driver
	6 	 	ToggleSwitch			-											T - Off/On, Mid/High & Mid/Low

TSwitch		Function										
-------		--------										
	1 	= 	High Vis Rear Light		12		Off/On								Low/High - LED driver
	2 	= 	5v Power On/Off			A3		(Circulating Warning Light?)			Transistor driven 5v power

	5	=	Debug - info output		USB		Display with TTY screen on PC (ArduinoIDE or PuTTY)
											Level   3=everything; 2= Power level & FlashingTrailerMarkerLights; 
													1=infrequent messages (legs, ...)
	6	=	Separate LED for TSwitch No.
									A0		Number of flashes =  TSwitch No.

Specials & Extras
		Leg Down Sensor				A1		Sensor to notify legs down			Linear Hall Effect Diode
		Trailer Brake				A5		Prop to Servo to control Brake		Proportional - Mid to High
		Running Lights (Back Right)	4		2 sets of side LEDs Off/On			Controlled by channel 2
					   (Back Left)	5
		Running Lights (Front)		7		2 side and 2 front LEDs Off/On		Controlled by channel 2 and 
																				flashed when Trailer Brake On

Suggested Change:  Move "pull up" for IR receiver to breakout board (then use single core sheilded!)
Lorry Idle = 210mA;  Camera & TX = 295mA.   (CL520 LED Driver pulls 0.5mA to control it.)
Trailer Running Lights = Back-6cm; 4 x 21cm; 7.5cm-Front.  Front Marker Lights = White < 1cm from side.
==============================================================

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
//#include <EnableInterrupt.h>

// -------------------------
//  Constants and variables to control functionality
//--------------------------
const int inputPin 			= 2;	// Input pin for PPM stream
//#define PPM_INPUT_TYPE	INPUT_PULLUP	// Input using embedded pull up resistor
#define PPM_INPUT_TYPE	INPUT		// Input using floating input
const int MAX_CHANNELS		= 8;	// number of channels being input in PPM stream
const int maxChannels		= 6;	// channels to process - can be less but not more that available input
#define PULSE_EDGE 		RISING		// Rising is better shape than falling
//  This controls the timing of the interrupt timer - too low and the system locks up so min 40?
//  It is effectively the number of micro seconds between interupts - too quick and all time is in here!!
const int timerPrecission = 40;
const int minProportionalValue = 700;	// = .7ms - shortest pulse = 1ms
						//debounce for input and means proportional channel for toggleSwitch channels
//const int channelDelayTimer = 50;	// to separate the outputs from input frame and each other

// Channel type - Proportional, Switch (all proportional output)
//				LED (via PWM) offToOn from center ignoring direction
//				lowercase l = LED (via PWM) offToOn for whole travel
// 		NOTE:  LED only available via some pins (PWM) depending on board/processor.
//				Toggle - only one with up to 10 switches.
const char channelType[] =   {"-PlSSSTSS"};  
//								12345678	// channel
//			1 = Legs
//			2 = Rear/Stop Lights
//			3 = Indicator 
//			4 = Indicator
//			5 = Reversing Light
//			6 = ToggleSwitch

const char invertChannel[MAX_CHANNELS+1] = {"---IIII--"}; 	// Invert or Normal ( I or - )
//											  12345678	// channel
const int channelPIN[MAX_CHANNELS+1] = {0,A4,6,0,0,0,0,0,0};	// channel pin
//		Proportional output - PWM		   1 2 3 4 5 6  7  8 //channel
const int channelDirectionPIN[MAX_CHANNELS+1] = {0,0,0,9,8,11,0,0,0};	// direction of channel pin 
//		Used for lights - turn PWM into on/off	   1 2 3 4  5 6 7 8	//channel
const int channelTimeLimit[MAX_CHANNELS+1] = {0,4500,0,0,0,0,0,0,0};		//millis (1000 = 1 second)
//const int channelTimeLimit[MAX_CHANNELS+1] = {0,0,0,0,0,0,0,0,0};
//		How long channel can stay not centered	   1 2 3 4 5 6 7 8 //channel
const int channelRateLimit[MAX_CHANNELS+1] = {0,200,500,500,500,500,500,500,500};	//0-500 for proportional
//			Max speed/rate for prop channels	  1   2   3   4   5   6   7   8 //channel
//			250 = half speed max  (channel time goes 1 - 1.5 - 2 msecs; hence the 500)
//			(used to slow down trailer leg up/down)
//			----------------------------------
const int channelSpecialType[MAX_CHANNELS+1] = {0,0,1,2,3,0,0,0,0};  //type
//		Type of processing for special things	  1 2 3 4 5 6 7 8    //channel
//				l - 1 = Turn running lights on when rear lights are on (but nor for brake lights!)
//				S - 2 = Flash rear running lights with indicator.
//				S - 3 = Flash rear running lights with indicator.

// ------- Running Lights ------------------
const int runningLightsPin1 = 7;		// front running lights - pin# (2 front side & front white marker lights)
const int runningLightsPin2 = 4;		// Rear Right Side Running Lights - pin#
const int runningLightsPin3 = 5;		// Rear Left Side Running Lights - pin#
const int runningLightsFlashPeriod = 1000;	// flash sequence every ....
const int runningLightsFlashInc = 100;		// increment for flash sequence
int runningLightsFlashPosition = 5;			// where in flash sequence
int runningLightsFlashCurrent = 0;			// current time - runningLightsFlashStart
unsigned long runningLightsFlashStart = 0;			// time previous sequence finished
unsigned long turnLeftStart = 0;
unsigned long turnRightStart = 0;

// -------- Legs -------------------
const int trailerLegsChannel = 1;
bool legsDirectionUp;
const int legsSensorPin = A1;
const int legsSensorDownValue = 860;

// ------- Trailer Brake ------------------
const int trailerBrakePin = A5;
static bool trailerBrakeOn = false;		// don't actually know but startup sets it so needs to be opposite here.
unsigned long trailerBrakeTimeOn;
const int trailerBrakeOnDelay = 2000;	// delay before brake is automatically tuned off

//static volatile unsigned int Failsafe[MAX_CHANNELS + 1]; 
				// array holding channel fail safe values
//static volatile decodeState_t State;         
				// this will be one of the following states

// ---- Toggle Switches ----- 
// 			First 4 are user control.  
//			5 (MAX_tSwitch-1) = Print Debug info. on console (via USB connection)
//			6 (MAX_tSwitch) = show ToggleSwitchNo. on PIN  
//									(as opposed to normal - combine SwitchNo. with Status LED (pin 13))
const int MAX_tSwitch = 6;		//x-2 for user, top 2 for system control
const int tSwitchPIN[MAX_tSwitch+1] 	= {0,12,A3,0,0,0,A0};	//pin
//											  1  2 3 4 5  6  //switch no.
//									 1 2 3 4 5 6  //switch no.
int tSwitchValue[MAX_tSwitch+1]	= {0,0,0,0,0,0,0};		//--- Set initial values of toggleSwitches here. =========
//int tSwitchValue[MAX_tSwitch+1]	= {0,0,0,0,0,3,1};	//--- Initial values ---
//====== DEBUG======== (set 5 & 6 on to help debug) ==================================================================
//====================================================================================================================
//				tSwitch Channel:	1 = High Vis Rear Light (trailer)
//									2 = Rotating Light (needs a RC (1.5/20mS) PWM input!!)
//									3 = ...
const int tSwitchType[MAX_tSwitch+1] = {0,0,0,0,0,0,0};		// type					// not used at present
//										  1 2 3 4 5 6  //switch no.
//				tSwitch Type:	0 = toggle on/off;  
//								1x = Proportional output;		(not used)
//								2x = PWM output					(not used)
//							x: 	0 = on/off-0/1500; 1 = low/high toggle; 
//								2 = 3 position toggle; 2 = high/mid toggle only; ....
int tSwitchNo = 1;					// initial switch no.
const int tSwitchSetTime = 750;		// milli seconds for off-on-off toggle to increment or set. (how quick are you??)
const int tSwitchLED_FlashOnTime = 100;
const int tSwitchLED_FlashOffTime = 400;
const int tSwitchLED_FlashGap = 1000;
static volatile int tSwitchValueOutput[MAX_tSwitch+1];	// copy of settings
unsigned long tSwitchTimerNow = 0;
unsigned long tSwitchTimerStartOFF = 0;
unsigned long tSwitchTimerStartON = 0;
unsigned long tSwitchLED_FlashTimeStart = 0;
int tSwitchLED_FlashPulseCount = 0;
unsigned long tSwitchLED_ThisTime = 0;

// --------------------------------
// Proportional settings
// --------------------------------
//int PULSE_LENGTH_MID_temp[5];	// dev version - for learning centre point of 2 stick value as transmitted
const int PULSE_LENGTH_MIN = 1000;		// min, mid and max pulse lengths
const int PULSE_LENGTH_MID = 1500;		
const int PULSE_LENGTH_MAX = 2000;
const int SWITCH_ON_SETTING = 1800;		//for Switch and Toggle use
const int SWITCH_OFF_SETTING = 1200;
const int SWITCH_MID_SETTING_A = 1300;
const int SWITCH_MID_SETTING_B = 1700;	
//const float PWM_MULTIPLIER   = .255;		// convert time (microS) to 0-255 for PWM output
//									// hard coded as "long( ....... * 255 / 1000) " to make integer arithmetic

// -------------------------
// for output cycles
//--------------------------
static volatile int channelTimeCopy[MAX_CHANNELS + 1];			// copy of channel times used to process
unsigned long channelTimeLimitStart[MAX_CHANNELS + 1];  		//time the channel started for time limit
static volatile int channelOutTimeStart[MAX_CHANNELS + 1];  	//time the channel started output
static volatile int tSwitchOutTimeStart[MAX_tSwitch+1];			//time the proportional tSwitch started output
unsigned long outputMicrosNow;
unsigned long outputMicrosDiff;
unsigned int outChannel = MAX_CHANNELS+10;		// make sure output doesn't start till frame received


// -------------------------
// for monitoring input and showing status via LED_BUILTIN
//		Pin13 (LED_BUILTIN) is on board LED = fast flash at start - no input signal
//--------------------------
int statusChannelTimeCopy[MAX_CHANNELS + 1];		// copy of channel times used for Debug
const int monLED_PIN = LED_BUILTIN;  //pin 13
int monLED_OnTime	= 10;	//flash time - On time 		// setting for initial flashing before signal found
int monLED_OffTime	= 50;	// Off
int monLED_Gap		= 150;  // Gap between sets of flashes
unsigned long monLED_ThisTime = 0;
unsigned long monLED_FlashTimeStart = 0;
unsigned long monLED_FlashPulseStart = 0;
int monLED_FlashPulse = 1;
int monLED_FlashPulseCount = 0;
unsigned long monTimeOfLastCycle = 0;	// i.e last output cycle started
unsigned long monTimeElapse = 0;		// working value to save calls to millis
int monLED_CycleCount = 0;
bool mon0 = true; bool mon1 = false;			// so that failure messages only appear once in output.
bool mon2 = false; bool mon3 = false; bool mon4 = false;
//int monPowerMin = 4950;		// minimum value of Power monitor before ..... (5120 with Diode via Vin; 4918 via USB)
int monPowerMin = 4850;
long monPowerReadingValue = 0;
int monPowerReadingCount = 0;
//====== DEBUG========
int debugCycleTime = 1500;	// 1.5 secs
unsigned long debugCyleStart = 0;
int debugLongCycleCount = 0;
int testCycleCount = 0;


// ----------------------------------------------------------
//for interruptReadFrame (input of frame of channel data)
//-----------------------------------------------------------
const int LONGEST_WAIT_FOR_CHANNEL = 3000;  //no more channels comming?  3 milli seconds (could be just > 2!)
static volatile int channelTime[ MAX_CHANNELS + 1];
				// array holding channel channelTime width value in microseconds
static volatile int channelIn = 0;      
				// number of channels detected so far in the frame (first channel is 1)
				// channel 0 is used for the length of the inter frame gap at end of frame
static volatile bool frameAvailable = false; // indicates a new frame of data has arrived
static volatile unsigned long nowTime;			//Times for channels in the frame.
static volatile unsigned long oldTime = 0;
static volatile unsigned long diffTime = 0;

// for testing / debug:
//static volatile uint16_t frameCount=0;
//static volatile uint16_t longFrameCount=0;
//static volatile uint16_t shortFrameCount=0;
//static volatile uint16_t frameInterruptCount=0;	// number of times through before data is dealt with


//  Emergency restart of sketch!!!!!  Things happen - should never be needed.
void(* resetSketch) (void) = 0;


void interruptReadChannels() {
	//500us pulses with time between rising edges = RC PWM time (1-2ms).
	nowTime = micros();			// get current time - microseconds
	diffTime = nowTime - oldTime;	// calculate RC PWM pulse length (time since last rising edge!)
	if (diffTime > minProportionalValue) {		// sort of debounce - if pulse shorter than 0.7ms it cannot have finished.
		oldTime = nowTime;
		channelIn++;
		if (diffTime >= LONGEST_WAIT_FOR_CHANNEL) {		// check for frame finishing - actually start of next frame!!!!
	//		frameCount++;
	//		if (channelIn < MAX_CHANNELS+1){
	//			shortFrameCount++;
	//		}
			channelIn = 0; 
		}
	//	if (channelIn > MAX_CHANNELS) {		// check for long frame 
	//		longFrameCount++;
	//	} else {
			channelTime[channelIn] = diffTime;		// capture RC PWM pulse length
	//	}
		if (channelIn == maxChannels) {		// we can start after the 6 channels we are using are here.
			frameAvailable = true;
		}
	}
}

static volatile int timerTime;
static volatile int outputIntDiff;
// -------------------------------
//for interrupt Timer (for output)
//--------------------------------
ISR(TIMER2_COMPA_vect){ 
	//interrupt every 40? micro seconds - controlled by constant timerPrecission
	timerTime = timerTime + timerPrecission;	
	// -------------------------------------------------------------------
	// ======== finish processing for all channels still active ==========
	// -------------------------------------------------------------------
	for (int i = 1; i <= maxChannels; i++) {
		if (!(channelTimeCopy[i] < 0)) {							// channel still active
			outputIntDiff = timerTime - channelOutTimeStart[i];
			if (outputIntDiff >= channelTimeCopy[i]) {			// finished?
				digitalWrite(channelPIN[i], LOW);
				channelTimeCopy[i] = -outputIntDiff;				// done
			}
		}
	}

/*
//	for (int i = 1; i <= MAX_tSwitch-2; i++) {			
	const int i = 1;		
	// output the user ToggleSwitch proportional values - not the system or on/off ones
		if (tSwitchValueOutput[i] > minProportionalValue) {			//for proportional channels still active
			outputIntDiff = timerTime - tSwitchOutTimeStart[i];
			if (outputIntDiff >= tSwitchValueOutput[i]) {		// finished?
				digitalWrite(tSwitchPIN[i], LOW);
				tSwitchValueOutput[i] = -outputIntDiff;			// done
			}
		}
//	}

*/

}

// -------------------------------
//		Setup
//--------------------------------
void setup() {
	Serial.begin(115200);
	if (tSwitchValue[MAX_tSwitch-1]) Serial.println("\n--- System Starting ---");
	pinMode(monLED_PIN, OUTPUT);
	channelTime[0] = -1;		// just something that will be noted if seen
	for (int i = 1; i <= maxChannels; i++){
		// set the output pins (hardware) as needed.
		if (channelPIN[i] > 0) pinMode(channelPIN[i], OUTPUT);
		if (channelDirectionPIN[i] > 0) pinMode(channelDirectionPIN[i], OUTPUT);
		// init the storage for input interrupt routine
		channelTime[i] = -2;  // negative = input processed
//		if (i <= 4) PULSE_LENGTH_MID_temp[i] = PULSE_LENGTH_MID;	// dev version for learning mid point
	}
	for (int i = 1; i <= MAX_tSwitch; i++){
		// set the output pins (hardware) as needed.
		if (tSwitchPIN[i] > 0) pinMode(tSwitchPIN[i], OUTPUT);
	} 
	pinMode(legsSensorPin, INPUT);								// Linear Hall Effect Sensor - no pullup needed
	pinMode(trailerBrakePin, OUTPUT);
	pinMode(runningLightsPin1, OUTPUT);
	pinMode(runningLightsPin2, OUTPUT);
	pinMode(runningLightsPin3, OUTPUT);
	
	bitWrite(TIMSK2, OCIE2A, 0); // disable interrupt
	// -- set timer2 interrupt  for interrupt timer - too low and the system locks up so min 40?
	//  timerPrecission is effectively the number of micro seconds between interupts 
	//	- too quick and processing doesn't finish!!  but the smaller the better for proportional accuracy.
	TCCR2A = 0;	// set entire TCCR2A register to 0
	TCCR2B = 0;	// same for TCCR2B
	TCNT2  = 0;	//initialize counter value to 0
		// set compare match register
		//  OCR2A = 200;  // (must be <256)
	OCR2A = 2*timerPrecission;	//  2 * 40   (must be <256)
	TCCR2A |= (1 << WGM21);	// turn on CTC mode
	TCCR2B |= (1 << CS21);    // Set CS21 bit for 8 prescaler;  22 for 64
//	TIMSK2 |= (1 << OCIE2A);	// enable timer compare interrupt
//	Don't enable here so timer doesn't start before first frame available
		//	TIMSK2 &= ~(1 << OCIE2A);	// disable interrupt
		// bitWrite(TIMSK2, OCIE2A, 0); // disable interrupt 
		// bitWrite(TIMSK2, OCIE2A, 1); // enable interrupt 	
			//byte timmer2Save = TIMSK2;	// save interrupt byte (current situation)
			//TIMSK2 &= ~(1 << OCIE2A);		// disable interrupt
			//TIMSK2 = timmer2Save;			// restore interrupt byte (old situation)

//	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);   // setup ADMUX for reading supply power value

/* -- Not needed with cheaper ESC --
	// Initialise legs so ESC gets a good centre position (else strange things can happen at hookup)
	int pulseTime = 1500;	// 1.5 millisecond pulses - i.e. mid position
	int pulseCount = 30; 	// 20+ pulses needed for ESC centre initialisation
	for (int i=1; i <= pulseCount; i++) {
		digitalWrite(channelPIN[1], HIGH); 	// set channel 1 (legs) high
		delayMicroseconds(pulseTime);		//	for xx time
		digitalWrite(channelPIN[1], LOW);	// set channel 1 (legs) low
		delay(18);							// wait for inter pulse gap
	}
*/
	
	// Initialise PPM input interrupt system
	pinMode(inputPin, PPM_INPUT_TYPE);  
	attachInterrupt(digitalPinToInterrupt(inputPin), interruptReadChannels, PULSE_EDGE);

	// Set trailer brake on.
	setTrailerBrake(true);			
	if (tSwitchValue[MAX_tSwitch-1]) Serial.println("##### Trailer Brake On at start-up"); //Debug 
	
} // end setup  

// -------------------------------
//		Main Loop
//--------------------------------
void loop() {

	// =========== start an output cycle =============
	if (frameAvailable == true) {
		// during inter frame gap and ready to output
		//  there is 20 minus 16 or so mS (i.e. the inter frame gap) to do all this.
		// first time through for this frame
		frameAvailable = false;		
//		delayMicroseconds(channelDelayTimer);		//------ to spread out starts ---------------------------
		outChannel = 1; 
//
		// Adjust legs value so that it doesn't wine when legs not in use.
		//channelTime[1] = channelTime[1]-40;
		channelTime[1] = channelTime[1]-60;		// 2 Jan 2023
		//
		channelTimeCopy[0] = channelTime[0];
		statusChannelTimeCopy[0] = channelTimeCopy[0];
		if (trailerBrakeOn && millis() - trailerBrakeTimeOn > trailerBrakeOnDelay) {	
													// seeing frames so turn trailer brake OFF after 2 seconds
			if (setTrailerBrake(false) && tSwitchValue[MAX_tSwitch-1]) 
										Serial.println("##### Trailer Brake Off when frame available"); //Debug on
		}			
		timerTime = 0;					// reset the time being used in the timer interrupt for next frame.
		//--Monitoring--
		monLED_CycleCount++;
		monTimeOfLastCycle = millis();
		mon1=true; mon2=true; mon3=true; mon4=true;
		//--Monitoring--end
		debugCycleTime = 1000;  // reset debugCycleTime to 1 second 
								// (set very slow if connection lost (no frames seen)
		bitWrite(TIMSK2, OCIE2A, 1); 	// enable output timer interrupt  (disabled for startup)
	}
	monTimeElapse = millis() - monTimeOfLastCycle;  


	// --------------------------------------------------------------
	// ========    processing for one channel at a time    ==========
	// --------------------------------------------------------------
	if (outChannel <= maxChannels) {
//		Serial.println(timerTime);
//		delayMicroseconds(channelDelayTimer);		//------ to spread out starts ---------------------------
		channelTimeCopy[outChannel] = channelTime[outChannel];
		if (invertChannel[outChannel] == 'I') 		// reverse direction of channel
					channelTimeCopy[outChannel] = ((-(channelTimeCopy[outChannel] - PULSE_LENGTH_MID))+PULSE_LENGTH_MID); 
/*
		// learn centre point of TX / RX for 4 stick channels.
		if (outChannel <= 4 && monLED_CycleCount > 10 && monLED_CycleCount < 60) {		// first 50 frames (1 second)
			PULSE_LENGTH_MID_temp[outChannel] = (PULSE_LENGTH_MID_temp[outChannel] + channelTimeCopy[outChannel]) / 2;
			//Serial.print("-- Channel: ");
			//Serial.print(outChannel);
			//Serial.print("  value: ");
			//Serial.print(PULSE_LENGTH_MID_temp[outChannel]);
			//Serial.println("");
		}
*/	
		// Process channel
		switch (channelType[outChannel]) {
		// 		set 'output high' on each proportional channel  and set others appropriately
		//			timing and 'output low' (end of PWM pulse) is done in timer interrupt
		//  P and S  require a pulse of the correct length output on the pin (proportional channel)
		//  L requires the output to be set for LED brightness by PWM (not a proportional channel)
		//  l (lowerCaseL) outputs LED brightness by PWM but using 1/2 input x 2 i.e. whole swing 
		//																	(direction pin available if needed)
		//  T  ToggleSwitch - needs a short or long pulse 	UP for control of switch number
		//													DOWN for on/off control of the switch
//--------------------
		case 'L':					// LED from centre
			//--set direction pin if there is one
			if (channelDirectionPIN[outChannel] > 0 && (channelTimeCopy[outChannel]-PULSE_LENGTH_MID) > 0 ) {
				digitalWrite(channelDirectionPIN[outChannel], HIGH);// direction on
			} else {
				digitalWrite(channelDirectionPIN[outChannel], LOW);// direction off
			}
			if (channelPIN[outChannel] > 0) {
				//--check for time limit
				if (channelTimeLimit[outChannel] > 0) {		//is there a limit
					if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A 
								&& channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {  //in centre (off)?
						channelTimeLimitStart[outChannel] = millis();  // init the start time
					} else {
						//is limit exceeded
						if (millis()-channelTimeLimitStart[outChannel] > channelTimeLimit[outChannel]) {
							channelTimeCopy[outChannel] = PULSE_LENGTH_MID;
						}
					}
				}
				//--check for rate limit
				if (channelTimeCopy[outChannel] > PULSE_LENGTH_MID + channelRateLimit[outChannel]) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MID + channelRateLimit[outChannel];    // limit rate
				}
				if (channelTimeCopy[outChannel] < PULSE_LENGTH_MID - channelRateLimit[outChannel]) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MID - channelRateLimit[outChannel];    // limit rate
				}
				analogWrite(channelPIN[outChannel], (long(abs(channelTimeCopy[outChannel]-PULSE_LENGTH_MID)) * 255/500));
				// convert from midpoint to PWM for LED =  1 to 255
			}
			statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel]; // copy data for debug/status output
			channelTimeCopy[outChannel] = -'L';			// channel dealt with set negative
			break;
//----------------
		case 'l':					//(lowerCase L) LED  from down to up (whole travel)
			// ----------------------
			// --- Special Type 1 ---
			// ----------------------
			// if > mid-switch A & < B (nearly in middle) - turn on extra "Specia1" output
			// so that the running lights are on when rear lights are on But not for just brake lights.
			// There are two output pins so front running lights can be flashed when trailer brake on.
			if (channelSpecialType[outChannel] == 1) {
				//special on if at middle
				unsigned long localTime = millis();
				if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A 
							&& channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {
					if (localTime - runningLightsFlashStart > 2000) digitalWrite(runningLightsPin1, HIGH);  // set on
					if (localTime - turnLeftStart > 500) digitalWrite(runningLightsPin2, HIGH);  // set on
					if (localTime - turnRightStart > 500) digitalWrite(runningLightsPin3, HIGH);  // set on
				} 
				//special off if at low  (can't use and "else" here - channel has 3 positions)
				if (channelTimeCopy[outChannel] < SWITCH_OFF_SETTING) {
					if (localTime - runningLightsFlashStart > 2000) digitalWrite(runningLightsPin1, LOW);  // set off
					digitalWrite(runningLightsPin2, LOW);  // set off
					digitalWrite(runningLightsPin3, LOW);  // set off
				}
			}
			if (channelPIN[outChannel] > 0) {
				//direction is not relevant 
				//		[- could be off/low or on/high (if >PULSE_LENGTH_MIN or >SWITCH_ON_SETTING)]
				//--check for time limit
				if (channelTimeLimit[outChannel] > 0) {		//is there a limit
					if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A 
							&& channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {  //in centre (off)?
						channelTimeLimitStart[outChannel] = millis();  // init the start time
					} else {
						//is limit exceeded
						if (millis()-channelTimeLimitStart[outChannel] > channelTimeLimit[outChannel]) {
							channelTimeCopy[outChannel] = PULSE_LENGTH_MID;
						}
					}
				}
				//--check for rate limit (use double value from PULSE_LENGTH_MIN)
				if (channelTimeCopy[outChannel] > PULSE_LENGTH_MIN + (channelRateLimit[outChannel]*2)) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MIN + (channelRateLimit[outChannel]*2); // limit rate
				}
				// force to max / min if close
				if (channelTimeCopy[outChannel] < SWITCH_OFF_SETTING) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MIN;
				}
				if (channelTimeCopy[outChannel] > SWITCH_ON_SETTING) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MAX;
				}
				// force to low middle if in middle (shows up on LED better)
				// for side/rear lights -  middle setting is too high (difference to full on is not enough)
				// full on and off is good  --  so just fix middle!!!
				if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A 
							&& channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {
	//				channelTimeCopy[outChannel] = PULSE_LENGTH_MID - 200;
					channelTimeCopy[outChannel] = SWITCH_MID_SETTING_A;
				}
				analogWrite(channelPIN[outChannel], (long(channelTimeCopy[outChannel]-PULSE_LENGTH_MIN) *255/1000));
								// convert to PWN for LED =  1 to 255
						}
			statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel];		// copy data for debug/status output
			channelTimeCopy[outChannel] = -'l';			// channel dealt with set negative
			break;
//----------------
		case 'P':					//proportional
			//--set direction pin						
			if (channelDirectionPIN[outChannel] > 0 && channelTimeCopy[outChannel] > PULSE_LENGTH_MID) {
				digitalWrite(channelDirectionPIN[outChannel], HIGH);// direction on
			} else {
				digitalWrite(channelDirectionPIN[outChannel], LOW);// direction off
			}
			if (outChannel == trailerLegsChannel && channelTimeCopy[outChannel] > PULSE_LENGTH_MID + 100) {
				legsDirectionUp = true;
			} else {
				legsDirectionUp = false;
			}
			if (channelPIN[outChannel] > 0) {
				if (outChannel == trailerLegsChannel && legsDirectionUp) {			// if legs going up set brakes OFF
					if (setTrailerBrake(false) && tSwitchValue[MAX_tSwitch-1]) 
											Serial.println("##### Trailer Brake Off when Legs going up"); //Debug on
				}
				// Check for legs fully down - if so stop - using Linear Hall Effect Sensor
				if (outChannel == trailerLegsChannel && !legsDirectionUp) {		// legs going down
					channelTimeLimitStart[outChannel] = millis();  // re-init the start time so time limit not used on down
					//Serial.println(analogRead(legsSensorPin)); 
					if (analogRead(legsSensorPin) > legsSensorDownValue) {		// legs fully down
						channelTimeCopy[outChannel] = PULSE_LENGTH_MID;			// force stick position to centre (off)
						if (setTrailerBrake(true) && tSwitchValue[MAX_tSwitch-1])	// set brakes ON
												Serial.println("##### Trailer Brake On when Legs Down"); //if Debug on
					}
				}
				//--check for time limit
				if (channelTimeLimit[outChannel] > 0) {		//is there a limit
					if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A 
								&& channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {  //in centre (off)?
						channelTimeLimitStart[outChannel] = millis();  // init the start time
					} else {	// not in centre (off)
						if (millis()-channelTimeLimitStart[outChannel] > channelTimeLimit[outChannel]) {	// is limit exceeded
							channelTimeCopy[outChannel] = PULSE_LENGTH_MID;			// force stick position to centre (off)
					//		if (outChannel == trailerLegsChannel && !legsDirectionUp) {		// legs will be fully down
					//			if (setTrailerBrake(true) && tSwitchValue[MAX_tSwitch-1])	// set brakes ON
					//								Serial.println("##### Trailer Brake On when Legs Down"); //Debug on
					//		}
						}
					}
				}

				//--check for rate limit
				if (channelTimeCopy[outChannel] > PULSE_LENGTH_MID + channelRateLimit[outChannel]) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MID + channelRateLimit[outChannel];    // limit rate
					if (outChannel == trailerLegsChannel) channelTimeCopy[outChannel] -= 50;		// legs need slower rate up!
				}
				if (channelTimeCopy[outChannel] < PULSE_LENGTH_MID - channelRateLimit[outChannel]) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MID - channelRateLimit[outChannel];    // limit rate
				}

				channelOutTimeStart[outChannel] = timerTime;
				digitalWrite(channelPIN[outChannel], HIGH);  //(proportional output (i.e. time based))
				statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel]; // copy data for debug/status output
			} else {
				statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel]; // copy data for debug/status output
				channelTimeCopy[outChannel] = -'P';	// channel not in use
			}
			break;
//----------------
		case 'S':					//Switch
					//only diff to P is output forced to HIGH, MID, LOW

			if (channelTimeCopy[outChannel] < SWITCH_OFF_SETTING) {
				channelTimeCopy[outChannel] = PULSE_LENGTH_MIN;
			} else if (channelTimeCopy[outChannel] > SWITCH_ON_SETTING) {
				channelTimeCopy[outChannel] = PULSE_LENGTH_MAX;
			} else {
				channelTimeCopy[outChannel] = PULSE_LENGTH_MID;
			}
			//--set direction pin
			if (channelDirectionPIN[outChannel] > 0) {
				if (channelTimeCopy[outChannel]-PULSE_LENGTH_MID > 0 ) {
					if (!digitalRead(channelDirectionPIN[outChannel])) digitalWrite(channelDirectionPIN[outChannel], HIGH); //on
					// --- Special Type 2&3 ---
					if (channelSpecialType[outChannel] == 2) {
						digitalWrite(runningLightsPin2, HIGH);
						turnLeftStart = millis();
					}
					if (channelSpecialType[outChannel] == 3) {
						digitalWrite(runningLightsPin3, HIGH);
						turnRightStart = millis();
					}
				} else {
					if (digitalRead(channelDirectionPIN[outChannel])) digitalWrite(channelDirectionPIN[outChannel], LOW); //off
					// --- Special Type 2&3 ---
					if (channelSpecialType[outChannel] == 2) {
						if (millis() - turnLeftStart < 100) digitalWrite(runningLightsPin2, LOW);
					}
					if (channelSpecialType[outChannel] == 3) {
						if (millis() - turnRightStart < 100) digitalWrite(runningLightsPin3, LOW);
					}
				}
			}
//========================================================================
//  put special type processing here - no channel and no direction means just this processing.
//========================================================================
			if (channelPIN[outChannel] > 0) {
				//--check for time limit
				if (channelTimeLimit[outChannel] > 0) {		//is there a limit
					if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A 
							&& channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {  //in centre (off)?
						channelTimeLimitStart[outChannel] = millis();  // init the start time
					} else {
						//is limit exceeded
						if (millis()-channelTimeLimitStart[outChannel] > channelTimeLimit[outChannel]) {
							channelTimeCopy[outChannel] = PULSE_LENGTH_MID;
						}
					}
				}
				//--check for rate limit
				if (channelTimeCopy[outChannel] > PULSE_LENGTH_MID + channelRateLimit[outChannel]) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MID + channelRateLimit[outChannel];    // limit rate
				}
				if (channelTimeCopy[outChannel] < PULSE_LENGTH_MID - channelRateLimit[outChannel]) {
					channelTimeCopy[outChannel] = PULSE_LENGTH_MID - channelRateLimit[outChannel];    // limit rate
				}
				//			channelOutTimeStart[outChannel] = micros();	
				channelOutTimeStart[outChannel] = timerTime;
				digitalWrite(channelPIN[outChannel], HIGH);  //(proportional output (i.e. time based))
				statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel]; // copy data for debug/status output
			} else {
				statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel]; // copy data for debug/status output
				channelTimeCopy[outChannel] = -'S';	// channel not in use
			}
			break;
//----------------
		case 'T':					//Toggle switch
			if (tSwitchTimerStartOFF == 0 && channelTimeCopy[outChannel] <= SWITCH_OFF_SETTING) {	// start OFF timmer
				tSwitchTimerStartOFF = millis();
				tSwitchTimerStartON = 0;		// empty ON timmer!
			} 
			if (tSwitchTimerStartON == 0 && channelTimeCopy[outChannel] >= SWITCH_ON_SETTING) {		// start ON timmer
				tSwitchTimerStartON = millis();	
				tSwitchTimerStartOFF = 0;		// empty OFF timmer!
			} 
			if (channelTimeCopy[outChannel] > SWITCH_MID_SETTING_A && channelTimeCopy[outChannel] < SWITCH_MID_SETTING_B) {
				channelTimeCopy[outChannel] = PULSE_LENGTH_MID;								// force mid point for switch
			} 
			tSwitchTimerNow = millis();
			//  ---- Switch No./Counter ----
			if (tSwitchTimerStartOFF > 0 && channelTimeCopy[outChannel] == PULSE_LENGTH_MID) {	// switch pulsed OFF
				if (tSwitchTimerNow - tSwitchTimerStartOFF < tSwitchSetTime) {			// short OFF = next switch
					tSwitchNo++;
					if (tSwitchNo > MAX_tSwitch) tSwitchNo = 1;		//loop round
				} else {																// long OFF = reset switch counter
					tSwitchNo = 1;
				}
				tSwitchTimerStartOFF = 0;							// empty OFF timmer!
			} 

			//  ---- Switch Value ----
			if (tSwitchTimerStartON > 0 && channelTimeCopy[outChannel] == PULSE_LENGTH_MID) {	
																						// switch pulsed ON
				if (tSwitchType[tSwitchNo] == 0) {			// switch type 0 = toggle on/off (and count to 3)
					if (tSwitchTimerNow - tSwitchTimerStartON < tSwitchSetTime) {	// short ON = toggle
						if (tSwitchValue[tSwitchNo]) {							// value on?
							if (tSwitchNo == MAX_tSwitch-1 || tSwitchNo == 3) {	// (switch 3 for testing mechanism!)
								if (tSwitchValue[tSwitchNo] == 3) {
									tSwitchValue[tSwitchNo] = 0;
								} else {
									tSwitchValue[tSwitchNo]++;
								}
							} else {
								tSwitchValue[tSwitchNo] = 0;
								if (tSwitchPIN[tSwitchNo]) digitalWrite(tSwitchPIN[tSwitchNo], LOW);   	// set OFF
							}
						} else {
							tSwitchValue[tSwitchNo] = 1;
							if (tSwitchPIN[tSwitchNo]) digitalWrite(tSwitchPIN[tSwitchNo], HIGH);  // set ON
						}
					} else {												// long ON  = turn OFF	?????????????????
						//tSwitchValue[tSwitchNo] = 0;						//			now used in tractor - gearbox control
						//if (tSwitchPIN[tSwitchNo]) digitalWrite(tSwitchPIN[tSwitchNo], LOW);   	// set OFF
					}
				}
				if (tSwitchType[tSwitchNo] == 10) {			// switch type 10 = Proportional output,  MID tggled on/off to 0
					if (tSwitchTimerNow - tSwitchTimerStartON < tSwitchSetTime) {	// short ON = toggle
						if (!tSwitchValue[tSwitchNo]) {
							tSwitchValue[tSwitchNo] = PULSE_LENGTH_MID;
						} else {
							tSwitchValue[tSwitchNo] = 0;
						}
					} else {														// long ON  = turn OFF
						tSwitchValue[tSwitchNo] = 0;
					}
//					delayMicroseconds(channelDelayTimer);		//------ to spread out starts ---------------------------
					// output to be high - proportional output (i.e. time based)
					tSwitchOutTimeStart[tSwitchNo] = timerTime;
					digitalWrite(tSwitchPIN[tSwitchNo], HIGH);
				}
				if (tSwitchType[tSwitchNo] == 11) {			// switch type 11 = Proportional output,  high/low
					if (tSwitchTimerNow - tSwitchTimerStartON < tSwitchSetTime) {	// short ON = toggle
						if (tSwitchValue[tSwitchNo] == PULSE_LENGTH_MAX) {
							tSwitchValue[tSwitchNo] = PULSE_LENGTH_MIN;
						} else {
							tSwitchValue[tSwitchNo] = PULSE_LENGTH_MAX;
						}
					} else {														// long ON  = turn OFF
						tSwitchValue[tSwitchNo] = PULSE_LENGTH_MIN;
					}
//					delayMicroseconds(channelDelayTimer);		//------ to spread out starts ---------------------------
					// output to be high - proportional output (i.e. time based)
					tSwitchOutTimeStart[tSwitchNo] = timerTime;
					digitalWrite(tSwitchPIN[tSwitchNo], HIGH);
				}
				if (tSwitchType[tSwitchNo] == 12) {			// switch type 12 = Proportional output,  high/mid/low
					if (tSwitchTimerNow - tSwitchTimerStartON < tSwitchSetTime) {	// short ON = toggle through
						if (tSwitchValue[tSwitchNo] == PULSE_LENGTH_MAX) {
							tSwitchValue[tSwitchNo] = PULSE_LENGTH_MIN;
							if (tSwitchValue[tSwitchNo] == PULSE_LENGTH_MIN) {
								tSwitchValue[tSwitchNo] = PULSE_LENGTH_MID;
							} else {
								tSwitchValue[tSwitchNo] = PULSE_LENGTH_MAX;
							}
						}
					} else {														// long ON  = turn OFF
						tSwitchValue[tSwitchNo] = PULSE_LENGTH_MIN;
					}
//					delayMicroseconds(channelDelayTimer);		//------ to spread out starts ---------------------------
					// output to be high - proportional output (i.e. time based)
					tSwitchOutTimeStart[tSwitchNo] = timerTime;
					digitalWrite(tSwitchPIN[tSwitchNo], HIGH);
				}
				if (tSwitchType[tSwitchNo] == 13) {			// switch type 13 = Proportional output,  high/mid only
					if (tSwitchTimerNow - tSwitchTimerStartON < tSwitchSetTime) {	// short ON = toggle
						if (tSwitchValue[tSwitchNo] == PULSE_LENGTH_MAX) {
							tSwitchValue[tSwitchNo] = PULSE_LENGTH_MID;
						} else {
							tSwitchValue[tSwitchNo] = PULSE_LENGTH_MAX;
						}
					} else {														// long ON  = turn OFF
						tSwitchValue[tSwitchNo] = PULSE_LENGTH_MID;
					}
//					delayMicroseconds(channelDelayTimer);		//------ to spread out starts ---------------------------
					// output to be high - proportional output (i.e. time based)
					tSwitchOutTimeStart[tSwitchNo] = timerTime;
					digitalWrite(tSwitchPIN[tSwitchNo], HIGH);
				}				
				tSwitchTimerStartON = 0;		// empty on timmer!
			}
			statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel]; // copy data for debug/status output
			channelTimeCopy[outChannel] = -'T';			// channel dealt with set negative
			break;
		}
//		statusChannelTimeCopy[outChannel] = channelTimeCopy[outChannel];		// copy data for debug/status output
		outChannel++;					// inc. channel being processed for output.
										// will do this for each channel - 1 at a time to allow finished processing below!
	}
	

/*	
// This was working but now using a timmer interrupt instead to get better (more consistent) finish times.
	// ------------------------------------------------------
	// ======== finish processing for all channels ==========
	// ------------------------------------------------------
	for (int i = 1; i <= maxChannels; i++) {
		if (channelTimeCopy[i] > 0) {								// channel still active
			outputMicrosNow = micros();
			outputMicrosDiff = outputMicrosNow - channelOutTimeStart[i];
			if (outputMicrosDiff >= channelTimeCopy[i]) {			// finished?
				digitalWrite(channelPIN[i], LOW);
				channelTimeCopy[i] = -outputMicrosDiff;				// done
			}
		}
	}
*/
// Used here as timing not critical
/* Not needed at present
	// ======== finish processing for ToggleSwitch channels ==========
	for (int i = 1; i <= MAX_tSwitch-2; i++) {			
	// output the user ToggleSwitch proportional values - not the system or on/off ones
		if (tSwitchValueOutput[i] > minProportionalValue) {			//for proportional channels
			outputMicrosNow = micros();
			outputMicrosDiff = outputMicrosNow - tSwitchOutTimeStart[i];
			if (outputMicrosDiff >= tSwitchValueOutput[i]) {		// finished?
				digitalWrite(tSwitchPIN[i], LOW);
				tSwitchValueOutput[i] = -outputMicrosDiff;			// done			
			}
		}
	}
*/

	// ----------------------------------------------------------------------
	// --  Trailer Brake flasher  --
	// ----------------------------------------------------------------------
	if (trailerBrakeOn) {
		if (runningLightsFlashPosition == 5) {
//Serial.println("--Brake4 ");
			runningLightsFlashPosition = 0;
			runningLightsFlashStart = millis();
			if (mon2 && tSwitchValue[MAX_tSwitch-1] >= 2) Serial.println("Trailer Brake On - Lights Flashing");
			runningLightsFlashCurrent = 0;
		} else {
			runningLightsFlashCurrent = millis() - runningLightsFlashStart;
		}
		if (runningLightsFlashCurrent > 
					runningLightsFlashPeriod + (runningLightsFlashInc * runningLightsFlashPosition)) {
//Serial.println("--Brake1 ");
			if ((runningLightsFlashPosition % 2)) {		// if odd
//Serial.println("--Brake2 ");
				digitalWrite(runningLightsPin1, HIGH);
			} else {
//Serial.println("--Brake3 ");
				digitalWrite(runningLightsPin1, LOW);
			}
			runningLightsFlashPosition++;
		}
	}

	// ----------------------------------------------------------------------
	// --  Monitoring  --
	// ----------------------------------------------------------------------

	// No input for 0.5	sec (some frames seen)
	if (mon0 && monLED_CycleCount > 50 && (monTimeElapse) > 500) {  // 50 frames seen + 0.5 sec 
		mon0 = false; mon1=true;
		// lost input signal - apply trailer brakes
		if (setTrailerBrake(true) && tSwitchValue[MAX_tSwitch-1]) 
								Serial.println("##### Trailer Brake On as lost connection"); //Debug on
//		}
	}
	// No input for 1 sec (some frames seen)
	if (mon1 && monLED_CycleCount > 50 && (monTimeElapse) > 1000) {  // 50 frames + 1 sec 
		mon1 = false; mon2=true; 
		// lost input signal - singal pulse
		monLED_OnTime = 50; monLED_OffTime = 100; monLED_FlashPulse = 1; monLED_Gap = 500; 
		if (tSwitchValue[MAX_tSwitch-1]) Serial.println("@@@@@@ Lost connection @@@@@@"); //Debug on
	}
	// No input for over 10secs  (some frames seen)
	if (mon2 && monLED_CycleCount > 50 && (monTimeElapse) > 10000) { // 50 frames + 10secs
		mon2 = false; mon3=true; 
		//double pulse
		monLED_OnTime = 50; monLED_OffTime = 100; monLED_FlashPulse = 2; monLED_Gap = 500; 
		if (tSwitchValue[MAX_tSwitch-1]) Serial.println("@@@@@@ Lost connection for 10 seconds @@@@@@"); //Debug on
	}
	// No input for over 5 mins  (some frames seen)
	if (mon3 && monLED_CycleCount > 50 && (monTimeElapse) > 300000) { // 5 mins
		mon3 = false;  mon4=true; 
		//back to fast blinking
		monLED_OnTime = 10; monLED_OffTime = 100; monLED_FlashPulse = 1; monLED_Gap = 100; 
		//reset count of frames seen
		monLED_CycleCount=0;
		if (tSwitchValue[MAX_tSwitch-1]) Serial.println("@@@@@@ Lost connection for 5 minutes @@@@@@"); //Debug on
	}
	// No input for over 10 mins
	if (mon4 && (monTimeElapse) > 600000) { // 10 mins
		// reset !!!!
		if (tSwitchValue[MAX_tSwitch-1]) Serial.println("@@@@@@ Lost connection for 10 minuts - serious!! @@@@@@"); //Debug on
		resetSketch();
	}
	//--Monitoring--end

	//----------------------------------
	// Flash onboard LED to show status and output Debug info if switch on.
	//----------------------------------
	if (monLED_FlashTimeStart == 0) {
		digitalWrite(monLED_PIN, HIGH);
		monLED_FlashTimeStart = millis();
		monLED_FlashPulseCount = 1;
	} else {
		monLED_ThisTime = millis();
		if (monLED_ThisTime - monLED_FlashTimeStart > 
							(monLED_OnTime * monLED_FlashPulseCount) + (monLED_OffTime * (monLED_FlashPulseCount - 1))) {
			digitalWrite(monLED_PIN, LOW);						// Off
		} 
		if (monLED_FlashPulseCount < monLED_FlashPulse) {
			if (monLED_ThisTime - monLED_FlashTimeStart > 
//							(monLED_OnTime * monLED_FlashPulseCount) + (monLED_OffTime * monLED_FlashPulseCount) {
							(monLED_OnTime + monLED_OffTime) * monLED_FlashPulseCount) {
				digitalWrite(monLED_PIN, HIGH);					// On 
				monLED_FlashPulseCount++;
			}
		}
		if (monLED_ThisTime - monLED_FlashTimeStart  > 
							monLED_Gap + (monLED_OnTime + monLED_OffTime) * monLED_FlashPulse) {  //reset flash
			monLED_FlashTimeStart = 0;
			
		}	
	}
	
	// flash LED with tagSwitch no. of flashes (so one can see which switch is for input)
	if (tSwitchValue[MAX_tSwitch]) {
		if (tSwitchLED_FlashTimeStart == 0) {
			digitalWrite(tSwitchPIN[MAX_tSwitch], HIGH);
			tSwitchLED_FlashTimeStart = millis();
			tSwitchLED_FlashPulseCount = 1;
		} else {
			tSwitchLED_ThisTime = millis();
			if (tSwitchLED_ThisTime - tSwitchLED_FlashTimeStart > 
							(tSwitchLED_FlashOnTime * tSwitchLED_FlashPulseCount) + (tSwitchLED_FlashOffTime * (tSwitchLED_FlashPulseCount - 1))) {
				digitalWrite(tSwitchPIN[MAX_tSwitch], LOW);				// Off
			} 
			if (tSwitchLED_FlashPulseCount < tSwitchNo) {
				if (tSwitchLED_ThisTime - tSwitchLED_FlashTimeStart > 
							(tSwitchLED_FlashOnTime + tSwitchLED_FlashOffTime) * tSwitchLED_FlashPulseCount) {
					digitalWrite(tSwitchPIN[MAX_tSwitch], HIGH);		//On
					tSwitchLED_FlashPulseCount++;
				}
			}
			if (tSwitchLED_ThisTime - tSwitchLED_FlashTimeStart  > 
							tSwitchLED_FlashGap + ((tSwitchLED_FlashOnTime + tSwitchLED_FlashOffTime) * tSwitchNo)) {  
				tSwitchLED_FlashTimeStart = 0;							//reset flash
			}
		}
		if (monLED_CycleCount > 50 && monTimeElapse < 500) {	// recently had a frame - 0.5 secs
			monLED_OnTime = 50; monLED_OffTime = 500; 
			monLED_FlashPulse = 1;  // use basic OK flash.
			monLED_Gap = 1500; 
		}
	} else {
		digitalWrite(tSwitchPIN[MAX_tSwitch], LOW);
		// set different build in LED flash
		if (monLED_CycleCount > 50 && monTimeElapse < 500) {	// recently had a frame - 0.5 secs
			monLED_OnTime = 50; monLED_OffTime = 400; 
			monLED_FlashPulse = tSwitchNo;  
			monLED_Gap = 1500;
		} 
	}
	
	if (millis() - debugCyleStart > debugCycleTime) {		//loop for serial input and debug output
		debugCyleStart = millis();
		
// Put debug like:	Serial.print(" -- 8indL: ");	here.

		if (Serial.available() > 0) {		// Set debug on or off from TTY connection
			String monSerialRead = Serial.readString();
			if (monSerialRead == "Debug1\n" || monSerialRead == "d1\n" || monSerialRead == "d1") {
				Serial.println("Debug is now turned on - Level 1");
				tSwitchValue[MAX_tSwitch-1] = 1;
			} else if (monSerialRead == "Debug2\n" || monSerialRead == "d2\n" || monSerialRead == "d2") {
				Serial.println("Debug is now turned on - Level 2");
				tSwitchValue[MAX_tSwitch-1] = 2;
			} else if (monSerialRead == "Debug\n" || monSerialRead == "d3\n" || monSerialRead == "Debug3\n" || monSerialRead == "d3") {
				Serial.println("Debug is now turned on - Level 3");
				tSwitchValue[MAX_tSwitch-1] = 3;
			} else if (monSerialRead == "tt\n") {
				Serial.println("Trailer Brake ON.");
				setTrailerBrake(true);
			} else if (monSerialRead == "t\n") {
				Serial.println("Trailer Brake OFF.");
				setTrailerBrake(false);
			} else {
					Serial.println("Debug is now turned off.  [reminder: Debugx]");
					tSwitchValue[MAX_tSwitch-1] = 0;
			}
		}	

		// get 10 - 12 readings and average
		monPowerReadingValue = monPowerReadingValue + readVcc();
		monPowerReadingCount++;

		if (debugLongCycleCount++ > 10) {
			debugLongCycleCount = 0;
			// Check Power status
			monPowerReadingValue = monPowerReadingValue / monPowerReadingCount;
			if ((monPowerReadingValue) < monPowerMin) {
				Serial.print("--Power reading value: ");
				Serial.println(monPowerReadingValue);
				if (tSwitchValue[MAX_tSwitch-1]) Serial.println("===================== Power too low ==========================");
				// All sorts starts here to show battery low.
				detachInterrupt(digitalPinToInterrupt(inputPin));	// turn off input
				setTrailerBrake(false);			// set trailer brake off
				bitWrite(TIMSK2, OCIE2A, 0);	// disable output timer
				for (int i=0; i<2000; i++) {
					digitalWrite(6, HIGH);	// Rear/Stop Lights
					digitalWrite(8, HIGH);	// Indicator Lights
					digitalWrite(9, HIGH);	// Indicator Lights
					digitalWrite(7, HIGH);	// Front Runnin Lights
					delay(100);
					digitalWrite(6, LOW);	// Rear/Stop Lights
					digitalWrite(8, LOW);	// Indicator Lights
					digitalWrite(9, LOW);	// Indicator Lights
					digitalWrite(7, LOW);	// Front Runnin Lights
					delay(200);
				}			
			}
			if (tSwitchValue[MAX_tSwitch-1] >= 1) {
				Serial.print("Legs position value: "); 
				Serial.println(analogRead(legsSensorPin)); 

				//Serial.print("--Power reading count: ");
				//Serial.print(monPowerReadingCount);
				//Serial.print("  ");
				Serial.print("Power reading value: ");
				Serial.println(monPowerReadingValue);
			}
			monPowerReadingValue = 0;
			monPowerReadingCount = 0;
		}
			
		// -- Output Debug info if required --
		if (tSwitchValue[MAX_tSwitch-1] >= 3 && monTimeElapse < 4000) {  // Debug ON
			Serial.print("Frames: ");				 // print the status of the channels & switches if Sw6 on
			Serial.print(monLED_CycleCount);	
			for ( int i = 0; i <= maxChannels; i++ ){
				Serial.print("\tCh");
				Serial.print(i);
				Serial.print(" ");
				Serial.print(statusChannelTimeCopy[i]);
				Serial.print(";  ");
			}
			Serial.println("");
			Serial.print("FrDiff: ");				 // print the status of the channels & switches if Sw6 on
			Serial.print(monLED_CycleCount);	
			for ( int i = 0; i <= maxChannels; i++ ){
				Serial.print("\tCh");
				Serial.print(i);
				Serial.print(" ");
//				Serial.print(channelTimeCopy[i]);

				if (channelType[i] == '-') {
					Serial.print(channelTimeCopy[i]);
				} else {
					if ((channelType[i] == 'P' && channelTimeCopy[i] != -80) || (channelType[i] == 'S' && channelTimeCopy[i] != -83)) {
						Serial.print(statusChannelTimeCopy[i] + channelTimeCopy[i]);
					} else {
						Serial.print("-'");
						Serial.print(char(-channelTimeCopy[i]));
						Serial.print("'");
					}
				}
	
				Serial.print(";  ");
			}
			Serial.println("");
			Serial.print("Switch no: ");
			Serial.print(tSwitchNo);
			Serial.print("   ");
			for (int i = 1; i <= MAX_tSwitch; i++) {
				Serial.print("\tSw");
				Serial.print(i);
				Serial.print(" ");
				Serial.print(tSwitchValue[i]);
				Serial.print(";  ");
			}							
			Serial.println("");	
			Serial.print("SwtchDiff: ");
			Serial.print(tSwitchNo);
			Serial.print("   ");
			for (int i = 1; i <= MAX_tSwitch; i++) {
				Serial.print("\tSw");
				Serial.print(i);
				Serial.print(" ");
				if (tSwitchValue[i] > minProportionalValue) {		//for proportional channels
					Serial.print(tSwitchValue[i] + tSwitchValueOutput[i]);
				} else {
					Serial.print(tSwitchValueOutput[i]);
				}
				Serial.print(";  ");
			}							
			Serial.println("");	
		}
	}
}

//  Function to set Trailer Brakes ON or OFF
bool setTrailerBrake(bool onOff) {
	int pulseTime;
	int pulseCount;
	if (onOff) trailerBrakeTimeOn = millis();
	if (onOff && !trailerBrakeOn) {
		pulseTime = 2000;
		pulseCount = 12;  // 10-15 pulses needed to turn brake on
		trailerBrakeOn = true;
		runningLightsFlashPosition = 4;
//		Serial.println("-- trailerBrakeOn = true;");
	} else if (!onOff && trailerBrakeOn) {
		pulseTime = 1500;
		pulseCount = 8;  // 7-8 pulses needed to turn brake off
		trailerBrakeOn = false;
		trailerBrakeTimeOn = 0;
//		Serial.println("-- trailerBrakeOn = false;");
	} else {
//		Serial.println("-- trailerBrakeOn = no change;");
		return false;
	}
	byte timmer2Save = TIMSK2;	// save interrupt byte (current situation)
	TIMSK2 &= ~(1 << OCIE2A);		// disable interrupt
//	bitWrite(TIMSK2, OCIE2A, 0); // disable interrupt so this bit works (only happens rarely) 
	for (int i=1; i <= pulseCount; i++) {
		digitalWrite(trailerBrakePin, HIGH); delayMicroseconds(pulseTime);
		digitalWrite(trailerBrakePin, LOW); delay(18);
	}
	TIMSK2 = timmer2Save;			// restore interrupt byte (old situation)
//	bitWrite(TIMSK2, OCIE2A, 1); // enable interrupt 
	return true;
}

long readVcc() {
	long result; // Read 1.1V reference against AVcc 
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // done in setup!!
	delay(2); // Wait for Vref to settle 
	ADCSRA |= _BV(ADSC); // Convert 
	while (bit_is_set(ADCSRA,ADSC)); 
	result = ADCL; 
	result |= ADCH<<8; 
	result = 1126400L / result; // Back-calculate AVcc in mV 

	return result; 
}

//--------------------------- end --- end --- end --- end ---------------------------------------------------------------
