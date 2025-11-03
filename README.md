This is a program written for the Arduino Uno R3 to control the Texas Instruments SN76489AN sound chip via serial MIDI. I'm fairly new to C and electronics, and this project was primarily a learning exercise, so please keep that in mind if you reference it for your own project.

Serial MIDI commands are sent to the Arduino via pin 0. I only own USB MIDI devices, so I've been converting USB messages to serial with a Raspberry Pi Pico, but I imagine connecting a classic 5-pin connector should work fine.

The Arduino communicates with the sound's data chip via pins 4, 5, 6, 7, 10, 11, 12, 13, and the chip's clock pin to pin 9. The chip's CE pin should be pulled down with a resistor, and the WE pin is not currently used.

The chip's noise channel can be controlled with predefined MIDI keys (defaults in parentheses). A beat track can be enabled and disabled (21), and have its BPM increased and decreased (22, 23). The noise channel can also be played directly (24, 26, 28).

