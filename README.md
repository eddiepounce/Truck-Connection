# Truck-Connection
MCU code to drive and decode IR connection for model RC trucks

Arduino Nano board used to capture status on Truck Tractor
and create a PPM data stream.

PPM data stream transfered to Trailer via IR through hollow King Pin of 5th Wheel.
Channels
      1 = Legs
			2 = Rear/Stop Lights
			3 = Indicator (L)
			4 = Indicator (R)
			5 = Reversing Light
			6 = ToggleSwitch (set of options, e.g. High Vis Rear Light, on one high/mid/low switch)

Arduino Nano board used to Decode PPM signal to drive legs motor, LEDs etc. via an interface board.



Electronics Schematics
https://easyeda.com/editor#id=6f724d5383ed4ccf9e811a0ded910573|c5105507989a4b489788ed4aebf3eb20|53d3a90703bc4d2fbbe508c8c92191dd
