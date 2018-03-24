Tingy Talkie project
========================

This project is derived from the original Thingy:52 Firmware and the purpose is to make a sort of walkie talkie using two Thingys.

The behaviour is modified so that at the startup the device not only activate the beaconing (advertising) but also start the discovery (scanning) and as soon as another Thingy is found a BLE connection is established.
This project implements the Thingy Sound Service Client that behaves similar to the Android app, which can send audio (writing the Speaker Characteristic) and activate the microphone for 'siniffing' audio (with notifications on Microphone Characteristic). Anyway not all the TSS Client features are implemented.

For the ThingyTalkie project, only the part related to audio transmission (i.e. Speaker Caracteristic) is used, since it is the local device which decides when to stream local audio to the remote one, not vice versa. The audio streaming is enabled just by pressing the button on the Thingy, when connected to another thingy the button works with a Push-To-Talk behaviour.

Another feature is that all the other parts of the firmware remain untouched, then firmware still works with the original Android App (I suppose it will work also on iOs but I didn't test it).

This project is developed during free time and no warranty is given about the stability of code.

For compiling the code follow the instruction given in the README.md, only GCC toolchain is supported.
 
