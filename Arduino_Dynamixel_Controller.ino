/*
Arduino_Dynamixel_Controller.ino
//20170512 initial version
//20170517 uS timing (was mS), vars are longs, i2C start/stop, and clock IN data w/.
//20201002 Returns valid JSON.
//20201005 James Wigglesworth updated to include Dynamixel Servo Support. See
 https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/ for instructions and connections. 
 Install Dynamixel libraries into Arduino IDE then "Sketch" / "Include Libraries" / "Manage Libraries" and 
 then search for Dynamixel. Select and then "Install" the DYNAMIXEL2Arduino and then DynamixelShield.
 Set the SERVO_ID, SERVO_MODE, and BAUD defines below as needed (pre-set to defaults).
 Note this changes the command serial port and requires a seperate USB/Serial adapter 
 on the DYNAMIXELShield UART RX/TX connector.
//20200203 James Newton / Tyler Skelton updated to support readback of servo position, torque, and 
 velocity and to set max "current" (torque(ish)) when moving.
//20211220 James Newton, add support for a step / direction stepper driver with velocity and accel. See
https://www.airspayce.com/mikem/arduino/AccelStepper/index.html for details. Install library via 
"Sketch" / "Include Libraries" / "Manage Libraries" and then search for "AccelStep". Select and then "Install"
//20211220 James Newton Clean up code and allow options to NOT support Dynamixel or Steppers via the defines

A simple Arduino script to set pins high, low, input, pull up, or drive analog / rc servo, 
Dynamixel servo, step / direction type stepper motor driver, clock out data with timing,
and read all or a single pin back... all via serial IO. 

Makes the Arduino: 
- a tool for generating test signals, and reading back results. 
Not as powerful as the busPirate, but more flexible in some ways and much easier to operate. 
- a generic IO controller which doesn't need a new program for each application.
Not a replacement for Firmata as this is intended to be used by a human directly 
via serial monitor or terminal, not from a program.

Commands:
#?   //return binary value of digital pin, and value for analog input if exists
     //if # and default # (set by comma command, see below) are zero or ommitted  
     //? returns all pins and analog values at once.
#I   //set pin # to an input. e.g. 3I
#P   //set pin # to an input with internal pullup. 4P
#H   //set pin # to a high output. 3H4H
#L   //set pin # to a low output. 5L4L3L
#D   //delay # microseconds between each command, with a minimum of about 47uS
#,   //comma. Saves # as the default for all commands e.g. 3,HLHLHLI
#,#A //set pin # to an analog output with value. Only PWM outputs will respond.
     // use with comma command e.g. 5,120A will put 120 on pin 5
_-   //low high clocked puts out the set of low and high signals shown on # with
     // a clock on #, e.g. 5,11-__-_--_ clocks out 10010110 on pin 11, with clock 
     // pulses on pin 5. Clock is currently falling edge only. 
.    //reads data back from # while clocking #, 
     // e.g. 5L 11H 5,11-__-_--_. ......... clocks out 10010110, gets the ack, and 
     // then 8 bits of data and a final ack.
(    //I2C start with # as SDA and #, as SCL
)    //I2C stop with # as SDA and #, as SCL. Pins left floating pulled up.
     // e.g. 5,11(-__-_--_. .........) starts, 10010110, gets ack, data, ack, stop
#,#R //Reboot / Initialize servo into mode. <id>,<mode>R. 
     // e.g. 1,4R starts servo id 1 in extended position mode
#,#S //Servo position. <id>,<degrees>S e.g. 2,90S moves servo id 2 to 90 degrees. 
#,#T //Torque setting. <id>,<percent>T e.g. 1,50T sets servo id 1 to half strength.
#,#M //stepper Motor. <step_pin>,<dir_pin>M 
#,#V //motion profile for the stepper. <accelleration>, <velocity limit>V
#G   //goto. Move the stepper motor to the specified position.
     
Commands can be strung together on one line; spaces, tabs, carrage returns and line feeds 
are all ignored. If no n is specified, value previously saved by , is used.
Examples:
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
1,4R50T90S
//Assuming an attached Dynamixel servo with an ID of 1, reset it to extended position mode, half
// strength, and move to 90 degrees. 
*/

//#define BAUD 115200
#define BAUD 57600
//The same baud rate should be used for both the Arduino and the Dynamixel. e.g. if you want to talk to the
//Arduino at 115200, then you need to re-program the Dynamixel to be at 115200. Of course, if you don't use
//the Dynamixels, you can set this to whatever you like. 

#define DYNAMIXEL_SUPPORT
//Note: If enabled, the Dynamixel Arduino shield is required and the standard Arduino UNO serial port is NOT functional! 
//You must install the USB to Serial interface on the Dynamixel Shield and use that for communication. RTFM

#define STEPPER_SUPPORT

#define ETO
//sending EOT can help OS serial device drivers return data instead of waiting forever for the file to end.
//https://stackoverflow.com/questions/50178789/signal-end-of-file-in-serial-communication


#ifdef DYNAMIXEL_SUPPORT
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

#define SERVO_ID 1

#define SERVO_MODE OP_EXTENDED_POSITION
//https://emanual.robotis.com/docs/en/popup/arduino_api/setOperatingMode/

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
int servo_id;
using namespace ControlTableItem;
#else
#define DXL_DIR_PIN -1
#define DEBUG_SERIAL Serial
#endif

#ifdef STEPPER_SUPPORT
#include <MultiStepper.h>
#include <AccelStepper.h>
//the default pins are used if you just start with an M oplet. Or 0,0M
#define DEFAULT_STEP_PIN 3
#define DEFAULT_DIR_PIN 4
/* Good starting values for different modes
 * Vel  Accel Microstep mode
 *  30   15   Full
 *  60   30   Half
 * 120   60   Quarter
 * 240  120   Eighth
 * 480  240   Sixteenth
 */
#define DEFAULT_VELOCITY 480
#define DEFAULT_ACCEL 240 
#define DEFAULT_STEP_DELAY_US 1

int dir_pin,step_pin;

void step(){ //TODO Allow for stepper drivers with a negative going step signal. Are there any?
    digitalWrite(step_pin, HIGH); // step HIGH
    delayMicroseconds(DEFAULT_STEP_DELAY_US);    // Delay the minimum allowed pulse width
    digitalWrite(step_pin, LOW); // step LOW
}

void step_forward() {
    digitalWrite(dir_pin, HIGH); // Set direction first else get rogue pulses
    step();
}

void step_back() {
    digitalWrite(dir_pin, LOW); // Set direction first else get rogue pulses
    step();
}

//AccelStepper stepper(AccelStepper::DRIVER, DEFAULT_STEP_PIN, DEFAULT_DIR_PIN); 
//Instead, use the version with the two functions, so we can control the pins.
//https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#afa3061ce813303a8f2fa206ee8d012bd
AccelStepper stepper(step_forward, step_back); //avoids setting up the pins now.
#endif

#define ANALOG_PINS NUM_ANALOG_INPUTS
#define DIGITAL_PINS NUM_DIGITAL_PINS 
//change above if you want ? to report fewer than actual pins.
#define CYCLE_DELAY 100

long n,p,d, radix;
int sign;
char cmd;

/*
https://playground.arduino.cc/Code/PwmFrequency
Pins 9 and 10 run at 31250Hz from Timer1 which is only used for servo. 
3 and 11 are on Timer2, 5 and 6 on Timer0. These are also used for delay, millis, etc...
So limiting our changes to Timer1 and pins 9 and 10 allows us to still have timing.
 switch(divisor) {
      case 1: mode = 0x01; break;   //31250Hz
      case 8: mode = 0x02; break;   //3906.25Hz
      case 64: mode = 0x03; break;  //488.28125Hz //default?
      case 256: mode = 0x04; break; //122.0703125Hz
      case 1024: mode = 0x05; break;//30.517578125Hz
    }
      TCCR1B = TCCR1B & 0b11111000 | mode;
https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
Says only Timer0 is used for delay and millis... 
We can try changing 3 and 11 on Timer2 
Different divisors, so compensate
T2  T1 / Freq
0x01 1 1 31250
0x02 2 8 3906.25
0x03 2 32  976.5625
0x04 3 64  488.28125 //default?
0x05 3 128 244.140625
0x06 4 256 122.0703125
0x07 5 1024  30.517578125
TCCR2B = TCCR2B & 0b11111000 | mode;
if (n>2) n--; //1,2,2,3,4,5,6
if (n>3) n--; //1,2,2,3,3,4,5
formula is f = clock / (510 * mode) where clock=16MHz
If you mess with TCCR0B, delay can be compensated as follows
0x01: delay(64000) or 64000 millis() ~ 1 second
0x02: delay(8000) or 8000 millis() ~ 1 second
0x03: delay(1000) or 1000 millis() ~ 1 second //default
0x04: delay(250) or 250 millis() ~ 1 second
0x05: delay(62) or 62 millis() ~ 1 second
(Or 63 if you need to round up.  The number is actually 62.5)
void setPin9_10PWMFreq(freq) {
  
}
*/

void delayus(unsigned long us) {
  if (us>10000) { //can't delayMicroseconds() more than 16838
    delay(us/10000);
    us=us % 10000;
    }
  delayMicroseconds(us);
  }

#ifdef DYNAMIXEL_SUPPORT
void rebootServo(int id, int mode) { //setup servo id number into mode.
  dxl.torqueOff(id);
  if (!mode) mode = SERVO_MODE;
  DEBUG_SERIAL.print("[{\"Servo\": ");
  DEBUG_SERIAL.print(id);
  if (dxl.setOperatingMode(id, mode)){
    DEBUG_SERIAL.print(", \"Mode\": ");
    DEBUG_SERIAL.print(mode);
    }
  if (dxl.writeControlTableItem(PROFILE_VELOCITY, id, 0)){ //0 is no velocity limit
    DEBUG_SERIAL.print(", \"Velocity\": \"MAX\"");
    }
  else { //can't set max velocity!
    DEBUG_SERIAL.print(", \"Velocity\": \"ERR\"");
    }
  if (dxl.torqueOn(id)){
    DEBUG_SERIAL.println(", \"Torque\": \"On\"");
    }
  DEBUG_SERIAL.println("}]");
#ifdef EOT
  DEBUG_SERIAL.write(04); //EOT
#endif
  }
#endif

void setup() {
  DEBUG_SERIAL.begin(BAUD);
  while(!DEBUG_SERIAL); //Wait until the serial port is opened
  DEBUG_SERIAL.println("[{\"Ready\": \"true\"}]");
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
#ifdef DYNAMIXEL_SUPPORT
  servo_id = SERVO_ID;
  dxl.setPortProtocolVersion(2.0);
  dxl.begin(BAUD);
  //rebootServo();
#endif
  n=0; //number
  p=0; //pin number
  d=2; //delay. Default is 2uS or 250KHz
  radix=10;
  sign=1;
#ifdef STEPPER_SUPPORT
  stepper.setAcceleration(DEFAULT_ACCEL);stepper.setMaxSpeed(DEFAULT_VELOCITY);
  dir_pin = DEFAULT_DIR_PIN;
  step_pin = DEFAULT_STEP_PIN;
#endif
  }

void loop(){
  while (DEBUG_SERIAL.available() > 0) { //if data has arrived
    int c = DEBUG_SERIAL.read(); //get the data
    if ('0' <= c && c <= '9') { //if it's a digit
      n = (c-'0')*sign + n*radix; //add it to n, shift n up 1 digit
      sign = 1; //in case it was negative
      continue; //and loop
      }
    cmd = char(c); //wasn't a number, must be a command
    if (' '==cmd || '\t'==cmd) { continue;} //whitespace does nothing
    if (','==cmd) { p=n; n=0; continue;} //save n to p, clear n, loop
    if (0==n) {
      if ('-'==cmd) {sign = -1; continue;} //if we don't have a number, it's a negative
      n=p; //if we don't have a value, use the prevous pin number. 
      //Note this means n can't be zero unless p is. 1,0 isn't possible, it becomes 1,1
      } 
    switch (cmd) {
      case '?': //get information
        DEBUG_SERIAL.print("{"); //optional, just to signal start of data
        if (0==n) { //if we didn't have a number selecting a pin
          DEBUG_SERIAL.print("\"?\":[\""); //optional, just to signal start of data
          for (int p = 0; p < DIGITAL_PINS; p++) { //get all the pins
            //n = digitalRead(p) + n<<1;   //convert to binary number
            if (DXL_DIR_PIN != p) { //don't mess with the servo pins
              DEBUG_SERIAL.print(digitalRead(p));//and also print.
              }
            }
          DEBUG_SERIAL.print("\"");
          //DEBUG_SERIAL.print(n); //print the binary value of all pins
          for (int p = 0; p < ANALOG_PINS; p++) { //also check all the analog
            if (DXL_DIR_PIN != p) { //don't mess with the servo pins
              DEBUG_SERIAL.print(",");
              DEBUG_SERIAL.print(analogRead(p));
              }
            }
#ifdef DYNAMIXEL_SUPPORT
          DEBUG_SERIAL.print(","); //add in data about the first servo.
          DEBUG_SERIAL.print(dxl.getPresentPosition(servo_id, UNIT_DEGREE));
          DEBUG_SERIAL.print(",");
          DEBUG_SERIAL.print(dxl.getPresentPWM(servo_id, UNIT_PERCENT));
          DEBUG_SERIAL.print(",");
          DEBUG_SERIAL.print(dxl.getPresentVelocity(servo_id, UNIT_RPM));
#endif
          }
        else { //specific pin
          DEBUG_SERIAL.print("\"");
          DEBUG_SERIAL.print(n);
          DEBUG_SERIAL.print("\":[");
          if (DXL_DIR_PIN != n) { //don't mess with the servo pins
            DEBUG_SERIAL.print(digitalRead(n)); //just that one pin
            if (ANALOG_PINS > n) { //if there is an analog channel
              DEBUG_SERIAL.print(",");
              DEBUG_SERIAL.print(analogRead(p)); //also return it
              }
            }
#ifdef DYNAMIXEL_SUPPORT
          else { //servo data
            DEBUG_SERIAL.print(dxl.getPresentPosition(p, UNIT_DEGREE));
            DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(dxl.getPresentPWM(p, UNIT_PERCENT));
            DEBUG_SERIAL.print(",");
            DEBUG_SERIAL.print(dxl.getPresentVelocity(p, UNIT_RPM));
            }
#endif
          }
        DEBUG_SERIAL.println("]}");
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
        //DEBUG_SERIAL.print("\n"); //new line can help after EOT to tigger xmit on OS serial handler
        break;
      case '-': 
      case 'H': //set pin n output high
        pinMode(n,OUTPUT);
        digitalWrite(n,HIGH);
        break;
      case '_': 
      case 'L': //set pin n output low
        pinMode(n,OUTPUT);
        digitalWrite(n,LOW);
        break;
      case 'I': //set pin n input
        pinMode(n,INPUT);
        break;
      case 'P': //set pin n input with pullup
        pinMode(n,INPUT_PULLUP);
        break;
      case '.': //clock in data from n via p
        pinMode(n,INPUT_PULLUP); //make n input with pull now
        break;
      case '(': //I2C start, data low while clock high
        pinMode(n,OUTPUT);
        digitalWrite(n,HIGH); //data high
        pinMode(p,OUTPUT); //setup clock (if not already)
        digitalWrite(p,HIGH); //send clock high
        delayus(d); //wait
        digitalWrite(n,LOW); //data low
        delayus(d); //wait
        digitalWrite(p,LOW); //send clock low
        continue; //no further processing
        break;
      case ')': //I2C stop, data low while clock high
        pinMode(n,OUTPUT); //data may be floating high (input from slave)
        digitalWrite(n,LOW); //so we need to drive it low
        pinMode(p,OUTPUT); //setup clock (if not already)
        digitalWrite(p,LOW); //send clock high
        delayus(d);
        pinMode(p,INPUT_PULLUP); //clock floats high
        delayus(d);
        pinMode(n,INPUT_PULLUP); //data floats high
        continue; //no further processing
        break;
      case 'A': //set pin p to analog output value n
        pinMode(p,OUTPUT);
        analogWrite(p,n);
        break;
      case 'D': //delay n ms per instruction
        d=n;
        break;
#ifdef STEPPER_SUPPORT
      case 'M': //<step>,<dir>M step, direction Motor stepper
        if (p) step_pin = p;
        if (n) dir_pin = n;
        DEBUG_SERIAL.print("[{\"Step\": ");
        DEBUG_SERIAL.print(step_pin);
        DEBUG_SERIAL.print(", \"Dir\": ");
        DEBUG_SERIAL.print(dir_pin);
        DEBUG_SERIAL.println("}]");
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
        pinMode(step_pin, OUTPUT);digitalWrite(step_pin, LOW);
        pinMode(dir_pin, OUTPUT);digitalWrite(dir_pin, LOW);
        p = n = 0;
        //break; //fall through and set default accel and velocity
      case 'V': //<acceleration>, <velocity>V Set acceleration and velocity
                //,<velocity>V Set default acceleration and velocity
        if (!p) p = DEFAULT_ACCEL;
        if (!n) n = DEFAULT_VELOCITY;
        stepper.setAcceleration(p + 1);stepper.setMaxSpeed(n);
        DEBUG_SERIAL.print("[{\"Accelleration\": ");
        DEBUG_SERIAL.print(p+1);
        DEBUG_SERIAL.print(", \"Velocity\": ");
        DEBUG_SERIAL.print(n);
        DEBUG_SERIAL.println("}]");
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
        break;
      case 'G': //<position>G  position Goto 
        stepper.moveTo(n);
        DEBUG_SERIAL.print("[{\"Goto\": ");
        DEBUG_SERIAL.print(n);
        DEBUG_SERIAL.println("}]");
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
        break;
/*
#E  pin Endstop. Input with Pullup. Run motor ccw until pin goes low. stepper.setMaxSpeed(SLOW_SPEED); stepper.moveTo(-MAX_LONG); if (!digitalRead(n)) stepper.stop();
 */
#endif
#ifdef DYNAMIXEL_SUPPORT
      case 'R': //<id>,<mode>S reboots servo.
        servo_id = p;
        rebootServo(p,n); 
        break; 
      case 'S': //<id>,<degrees> sets goal position. e.g. 1,90S 100000D 45S
        if (dxl.setGoalPosition(p, n, UNIT_DEGREE)) {
          DEBUG_SERIAL.print("[{\"Servo\": ");
          DEBUG_SERIAL.print(p);
          DEBUG_SERIAL.print(", \"Goal\": ");
          DEBUG_SERIAL.print(n);
          DEBUG_SERIAL.println("}]");
          }
        else { //can't move!
          DEBUG_SERIAL.println("[{\"Error:\" \"ServoPosition\"}]");
          }
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
        break;
      case 'T': 
        if (dxl.setGoalPWM(p, n, UNIT_PERCENT)) {
          DEBUG_SERIAL.print("[{\"Servo\": ");
          DEBUG_SERIAL.print(p);
          DEBUG_SERIAL.print(", \"Torque\": ");
          DEBUG_SERIAL.print(n);
          DEBUG_SERIAL.println("}]");
          }
        else { //can't set torque!
          DEBUG_SERIAL.println("[{\"Error:\" \"ServoTorque\"}]");
          }
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
        break;
#endif
      case '\n': 
      case '\r':
        n=0; cmd=0; //clear command and value at end of line.
        continue; //loop now, no delay
        break; //shouldn't get here
      default:
        DEBUG_SERIAL.print("\"");
        DEBUG_SERIAL.print(n);
        DEBUG_SERIAL.print(cmd);
        DEBUG_SERIAL.println("?\"");
#ifdef EOT
        DEBUG_SERIAL.write(04); //EOT
#endif
      }
    if ('0'>cmd || '_'==cmd) {//was it punctuation?
      digitalWrite(p,HIGH); //raise the clock
      pinMode(p,OUTPUT); 
      delayus(d/2); //half delay
      if ('.'==cmd) {DEBUG_SERIAL.print(digitalRead(n));}
      digitalWrite(p,LOW); //drop the clock
      delayus(d/2); //half delay
      }
    else {
      n=0; //zero out value for next command.
      delayus(d); //wait a bit for the next cycle.
      }
    //p is NOT cleared, so you can keep sending new commands only
    cmd=0; //done with command.
    }
#ifdef STEPPER_SUPPORT
  stepper.run(); //run full speed to support stepping.
#else
  delayus(CYCLE_DELAY); //save a little energy if we don't have characters. 
#endif
  }
