# ABC Device Peripheral

This is just an Arduino script, targeting the pi pico, to set pins high, low, input, pull up, or analog/rc servo, and control steppers, dynamixels, and more. And a bit more.

- Not something you have to re-program to support different devices; Everything is already there. It's a REPL; a live environment. You type instructions, the hardware / devices responds. However, it can use the EEPROM to remember and re-play instructions, which are pretty capable. 

- Not a replacement for Firmata as this is intended to be used by a human directly via serial monitor or terminal, in addition to being useful from a Pi or other high level robot controller. 

**LANGUAGE**: This branch is the language version of the controller. The language goals are:
- Get close to a high level language, w/ understandable syntax, without including a compiler, or a complex interpreter.
- Use the minimum resources possible to interpret the bytecodes, focus on io with the devices. 
- Follow the pattern: *Destination* [*Operation* *Source* ...], e.g. <BR>`a=b+c*d` (no operator presidence. a is (b+c)\*d, not b+(c\*d))
- The regular keywords and variables are single characters, making use of punctuation. e.g. `if` is `?`, `return` is `.`


Note: If SERVO_SUPPORT is enabled and the Dynamixel Shield is installed, then it does NOT communicate via the Arduino USB adapter during normal operation, it uses an external USB / TTL serial adapter instead because the Dynamixel is on the main Arduino serial port. If you don't need the servos, it will (probably) work with just the standard Arduino serial interface.

## DEMO:

Get a pi pico, hold down the button, plug in the USB, release the button, drag and drop the .uf2 from the release on to it. 
(or use whatever Arduino you like if you re-compile it... might work with an Uno if you disable the options)

**GPIO:** Now open a terminal and type. Want the LED on? type: `13H` or whatever pin the LED is on followed by 'H' for "high". Turn it off with `13L`. 

Read pin 3 with `t:3I` which sets pin 13 as an input, reads it, and sends the result to the terminal, stored by default in 't'. Use `U` instead of `I` to enable pullup. Colon ':' is "assign"... equal '=' is comparison never assignment.
```
6A=100?t:"YES"!t:"NO"
```
A for analog input, ? is the conditional bang '!' is else. 

`4P:128` does an "analog" (actually PWM) out at 128 out max PWM.

**RC SERVOS**: But you can also talk to more complex hardware. e.g. `2R:90` sets up a servo output on pin 2 and runs it to 90 degrees. But then it will jerk a bit if you re-issue that command with `2R:91`. Better to setup a device: 
```
a:D('R',2)
```
and then 
```
a:90
a:91
```
doesn't jerk. 

**REGISTERS**: The 'a' there is a register, which can hold values, references, or devices. Lower case letters are registers, but some have special functions so be careful: 
- 't' is the terminal, input or output.
- 's' is the stack pointer. Yes, you can crash it.
- 'r' sets the radix (default 10) and a-f or higher can turn into digits if r:16 for example then you are putting in hex values and f is 15, not the f register.
- 'q' sets the input buffer index for terminal input and matches (more on that later)
- 'p' is the program counter, you can jump and crash. 

e-o are (currently) safe, and u-z. 

**DEVICE DRIVERS**: Complex devices are setup via 'D'rivers which take in a type letter, and other parameters, usually the pin or pins, and whatever else is needed. For example, if you want to run run stepper motors
`m:D('S', 3, 4, 1000, 5000)`
pin 3 is step, 4 direction, and a max velocity of 1000 steps per second, 5000 acceleration. `m:500` to move 500 steps.

Support is coming for I2C devices, encoders, and so on. 

## Commands
see the [source code comment block](src/main.cpp) it's changing too fast to put here yet. 


### BUILD:

The release code is a defuault and you don't need to build to use that. 

1. If you want the SERVO_SUPPORT: Install Dynamixel libraries into Arduino IDE (must be 1.8.5 or up) via menu: "Sketch" / "Include Libraries" / "Manage Libraries" and then search for Dynamixel. Select and then "Install" the DYNAMIXEL2Arduino and then DynamixelShield 
2. If you want the STEPPER_SUPPORT: Install the [AccelStep](https://www.airspayce.com/mikem/arduino/AccelStepper/index.html) library into the Arduino IDE via menu: "Sketch" / "Include Libraries" / "Manage Libraries" and then search for AccelStep. Select and then "Install" it.
3. git clone to the PlatformIO project folder and open in VSCode with their extension.
4. Modify the BAUD value around line 50 to what your servo uses (default is 57600) or whatever you like if you aren't using the servos, SERVO_ID to match the servo address (default is 1) and SERVO_MODE to match the mode (default is the extended position mode which might not be supported on your servo)
5. Connect the Arduino /without the shield/ and program it. 
6. If you want the SERVO_SUPPORT: Connect the TTL serial adapter TTL Serial USB adapter to J3 on the shield. This is the small 4 pin connector off by itself near the green power terminal. 

Shield J3 pin | TTL adapter pin
-----------|----------------
G | GND
V | don't connect
R | TX
T | RX

7. Cut the DC volt adapter wires and strip them, then /carefully/ test to see which wire is positive and which negative before connecting to the shield. Getting that backwards will cost rather a lot. You should also be able to just use the barrel connector on the Arduino if you have the right adapter for that. Again, the voltage should match the servo's requirements.
8. Plug the shield onto the Arduino.

### OPERATE:
1. If you are using the Dynamixel Shield, check that the SW2 "UART" switch is in "DYANMIXEL" vs "UPLOAD" (see top right of picture below, switch is toward top of picture) and plug the TTL Serial adapter into your PC USB. Otherwise, (assuming you commented out the `#define SERVO_SUPPORT`) just plug in to the Arduino like always. In either case, you will need to configure a serial terminal program like Arduino Serial Monitor, PuTTY, or RealTerm, or whatever to 57600 N 8 1 (or whatever BAUD was set to in the code) on whatever port that shows up on.
2. Turn on the AC adapter and verify that the Arduino power LED comes on. Press the reset switch on the Arduino. You should see a ready message on your terminal program. Send `?` to test communications and ensure you get back a response. Send a command to turn the LED on or off, e.g. `13H` and `13L` should be the pin on the Uno. 
3. To use a Dynamixel, plug in a servo, and turn on the SW1 servo "POWER" switch. Check that the Servo LED blinks. (see bottom right of picture below, switch is toward top of picture). Reset the servo with it's id number and desired mode, e.g. `d:D('D', 1, 115200)` and verify the servo blinks. Now enter e.g. `d:90` and verify the servo moves to 90 degrees. 
4. To use a stepper driver, (assuming you commented IN the `#define STEPPER_SUPPORT` and installed the library) connect ground to ground, then connect the step line to one of the available pin, e.g. 10 and step to another, e.g. 11. Power on the stepper driver and motor, then send e.g. `m:D('S', 10, 11, 1000, 5000)` to initialze the system (the 1000, 5000  set speed and acceleration) then use e.g. `m:500` to move the motor a bit. 

<img src="https://user-images.githubusercontent.com/419392/89340621-de190500-d654-11ea-8f35-cad97d78e372.png">

### Pins 
Note that the Dynamixel shield uses pins 0, 1, 2, 7, and 8. Normally, an Arduino will communicate with the PC on pins 0 and 1, but those are used to talk to the servoes, so 7 and 8 are used for communications with the PC via the USB adapter instead. The built in USB adapter on the Arduino is only used for uploading the sketch and is then rendered useless. Pin 2 is also used by the shield. All the other pins available on the Arduino are unused and available for your applications.

Analog IO pins actually start numbering after the digital IO pins. e.g. on an Arduino Uno, A0 is pin 14 because the last digital pin is 13. A0 is the name of the pin, 14 is its number (on an Uno). 

<img src="https://emanual.robotis.com/assets/images/parts/interface/dynamixel_shield/pinmap.png">
