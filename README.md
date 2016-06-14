# MIDIBLE101

Worked hard on figuring out MIDI over BLE for the Arduino 101 while I was competing on America's Greatest Makers. I figure y'all should get a jump start. 
These projects were tested and verified to work with iOS and OSX. 

The MinimalMIDIBLE project is everything you need to get started sending MIDI from an Arduino 101. The sketch outputs random MIDI notes to test out the system. This is a great place to start if you want to add wireless MIDI to a project. You can even add MIDI jacks to the arduino and make a DIN-MIDI to BLE converter (all the rage right now). 
Receiving MIDI is being worked on. 

The Airpeggiator project is an arpeggiator that you can control in the air using the Intel Curie's IMU. It calculates the roll and pitch and turns it into MIDI notes. I added in a quantizer with a few predefined scales. At the top of the sketch, you can set the key, scale, and tempo of the arpeggiator. 
I think this project serves as a pretty good jumping point to music creation. 

HardwiredMIDI fixes issues with the stock Arduino MIDI example not working on the Curie (Arduino 101). The fix is to use software serial and specify pins 0 and 1 for the TX/RX.

MIDIReceiveEventHandler receives MIDI data over BLE. **Note, this is still buggy as of 6/14/2016** It then forwards that data out of the hardwired serial TX. I used this MIDI shield for testing: http://linksprite.com/wiki/index.php5?title=MIDI_Shield_for_Arduino


Enjoy. Let me know if you have any questions or ideas or want to show off cool projects you did with this code. 

oren@auxren.com
