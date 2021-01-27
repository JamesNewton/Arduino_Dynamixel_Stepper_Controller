# Arduino Dynamixel Controller
USB adapter to Arduino with Dynamixel Shield provides simple digital IO and motion. 

This is a "kitbash" of this gist:
https://gist.github.com/JamesNewton/8b994528ff3ce69e60bbb67c40954cd2
with some firmware changes to support the Dynamixel servo via the hardware in this repo:
https://github.com/JamesNewton/Dynamixel-Shield-Setup

Note: It does NOT communicate via the Arduino USB adapter during normal operation, it uses an external USB / TTL serial adapter instead because the Dynamixel is on the main Arduino serial port. 


Simple Arduino script to set pins high, low, input, pull up, or analog/servo, 
clock out data with timing, and read all or a single pin back via serial IO. 
Written for the tiny-circuits.com TinyDuino in the end effector of the 
Dexter robot from HDRobotic.com, but generally useful to turn the Arduino
into a tool for generating test signals, and reading back results. Not as 
powerful as the busPirate, but more flexible in some ways and much easier to
operate. Not a replacement for Firmata as this is intended to be used by a 
human directly via serial monitor or terminal, not from a program.

## Commands

All commands are in the format [#,][#]L where [#,] is an optional saved number, [#] is a primary number, and L is an opcode. 

Op   | Description
---- | ---
#?   | Return binary value of digital pin, and value for analog input if supported by the pin. If # and default # (set by comma command, see below) are zero or ommitted, ? returns all pins and analog values at once. e.g. `1?` might return `{"1":[1,459]} ` indicating the pin is digital high, and analog 459 counts.
#I   | Set pin # to an input. e.g. 3I
#P   | Set pin # to an input with internal pullup. 4P
#H   | Set pin # to a high output. 3H4H
#L   | Set pin # to a low output. 5L4L3L
#D   | Delay # microseconds between each command, with a minimum of about 47uS
#,   | Comma. Saves pin # as the default pin for all commands e.g. 3,HLHLHL
#A   | Set pin # to an analog output with value. Only PWM outputs will respond. Use with comma command e.g. 5,120A will put 120 on pin 5
#S   | Servo angle. Send number then S to set the servo to that position.
`_-`   | "low high clocked" Puts out a set of low and high signals on # with a clock on #, e.g. `5,11-__-_--_` clocks out 10010110 on pin 11, with clock pulses on pin 5. Clock is currently falling edge only. `5,11-` is basically `5L11H5H5L11L`
`.`    | reads data back from # while clocking #, e.g. `5L 11H 5,11-__-_--_. .........` clocks out 10010110, gets the ack, and then 8 bits of data and a final ack.
(    | I2C start with # as SDA and #, as SCL
)    | I2C stop with # as SDA and #, as SCL. Pins left floating pulled up. e.g. `5,11(-__-_--_. .........)` starts, 10010110, gets ack, data, ack, stop

Commands can be strung together on one line; spaces, tabs, carrage returns and line feeds 
are all ignored. If no n is specified, value previously saved by , is used.

### Examples
````
?
//returns something like: {"?":["10010000001111",739,625,569,525,493,470]}
// where 10010000001111 shows the binary value of each pin, from 0 to 14. Pin 0 is first
// 739,625,569,525,493,470 are the values read from each analog channel 0 to 5

1?
//returns something like: {"1":[1,459]} where 1 is the binary value of pin 1 and 
//459 is the analog value of channel 1

6?
//returns something like {"6":[0]} which is the value of pin 6 (no analog)

4L 6H 5,120A
//(nothing returned) Drives pin 4 low, pin 6 high and puts a PWM / Analog value of 120 on pin 5
//this also saves pin 5 as the default pin for all commands from now on

240A
//(nothing returned) assuming prior command was 5,120A put 240 out pin 5 as new analog value

90S
//(nothing returned) move the attached Dynamixel servo to 90 degrees.

?
//assuming 5, has been recieved before, returns just the value of pin 5 and analog 5

0,
//(nothing returned) clears saved pin, ? now returns all pins.

1000D 5,LHLHLHL
//(nothing returned) delay is 1 millisecond between commands. So pin 5 pulse 3 times at ~200Hz
//Actually about 1.04mS because of the time it takes to recieve and interpret each command. 
//The delay command is also useful for making sure all the commands on a line arrive before they are 
//excecuted. e.g.:
10000D 3D 5,LHLHL?
//will put out 2 pulses at 50uS per pulse or 10KHz without the 10000D, they are 163uS
//The time it takes to interpret a command is about 47uS so 3D makes it 50. For 100uS
//53D would work. Take the uS delay you want and subtract 47.
//With larger delays, the error is consistant but has relativly less effect.
// Note that the CYCLE_DELAY is not used as long as new characters are available.
````

### BOM
qty | item | price | source
----|------|-------|---------------
1   | Arduino | ~$25 | Arduino.cc (please support Arduino by buying real parts?)
1   | Shield  | $20  | http://en.robotis.com/shop_en/item.php?it_id=902-0146-000
1   | TTL Serial USB adapter | ~$2 | Google CP2102 or e.g.<BR> https://www.amazon.com/Diymore-Converter-Support-Windows-Arduino/dp/B0776T51YT/ref=sr_1_9
1   | 12V adapter | ~$9 | Google AC/DC power adapter. e.g. <BR> https://www.amazon.com/Converter-Cigarette-Lighter-110-240V-Adapter/dp/B07DWXRD5F/ref=sr_1_4
1   | 4 pin JST cable | $? | ???

See: 
https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/ 
for instructions and connections. 

### BUILD:
1. Install Dynamixel libraries into Arduino IDE (must be 1.8.5 or up) via menu: "Sketch" / "Include Libraries" / "Manage Libraries" and then search for Dynamixel. Select and then "Install" the DYNAMIXEL2Arduino and then DynamixelShield 
2. Download the .ino file above and place in folder of the same name under Arduino folder. e.g. Documents\Arduino\DynamixelShieldSetup\DynamixelShieldSetup.ino and open it. 
3. Modify the NEW_BAUDRATE value around line 50 to the one you want. 
4. Connect the Arduino /without the shield/ and program it. 
5. Connect the TTL serial adapter TTL Serial USB adapter to J3 on the shield. This is the small 4 pin connector off by itself near the green power terminal. 

Shield J3 pin | TTL adapter pin
-----------|----------------
G | GND
V | don't connect
R | TX
T | RX

6. Cut the 12 volt adapter wires and strip them, then /carefully/ test to see which wire is positive and which negative before connecting to the shield. Getting that backwards will cost rather a lot. 
7. Plug the shield onto the Arduino.

### OPERATE:
1. Plug the TTL Serial adapter into your PC USB. You will need to configure a serial terminal program like PuTTY, or RealTerm, or whatever to 57600 N 8 1 (or whatever BAUD was set to in the code) on whatever port that shows up on. 
2. Plug in a servo, and turn on the SW1 servo "POWER" switch. 
3. Check that the SW2 "UART" switch is in "DYANMIXEL" vs "UPLOAD" (see top right of picture below, switch is toward top of picture)
4. Turn on the AC adapter and verify that the Arduino power LED comes on. You should see `[{"Ready": "true"}]` on your terminal program.
5. Check that the Servo LED blinks. (see bottom right of picture below, switch is toward top of picture)
6. Enter e.g. `90S` and verify the servo moves to 90 degrees



<img src="https://user-images.githubusercontent.com/419392/89340621-de190500-d654-11ea-8f35-cad97d78e372.png">

<img src="https://emanual.robotis.com/assets/images/parts/interface/dynamixel_shield/pinmap.png">