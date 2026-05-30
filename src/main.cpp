#include <Arduino.h>

/*
Goals 
- get close to a high level language, w/ understandable syntax, without including a compiler, 
- using the minimum resources possible to interpret the bytecodes. 
- The main pattern is: Destination, [Operation, Source ...], e.g. a=b+c-d. 
- The regular keywords in a language are single characters, making use of punctuation

0-9 (and lowercase a and on when radix is over 10) these accumulate base-10 digits into num 
a-z Variables/Registers (SRC or DST). Some registers have special meanings:
   r Radix (Default 10)
   p Program Counter aka PC
   s Stack Pointer aka SP.
  Might not be contiguous in the array. e.g. while a is 0. b could be register 4, c 8, etc..
:	Copy operation, e.g. "define" a:5 sets register 0 to 5. a:b copies the value of b to register 0.
	NUM may be added to the register address first. 3a could be the third byte after the start of a. 
@	index. Replace SRC or DST with the value at that address
	 and clear op. This sets the stage for another op and SRC.
	e.g. b@a sets the DST to the address of b plus the value of a.
	If the SRC is a port or port pin, read that value in. 
' (Single Quote) ASCII literal. Next character is its numeric decimal value ('A' is 65).
"	(Quote) Text. Each following char is copied to the DST until the ending quote.
	If the DST is a variable, the chars are actually copied into FLASH and the var is
	set to the starting address of the string in FLASH.
	If the operation was already " when a new starting " is seen, 
	put a " to the dest then enter text mode. "Push ""START""" prints Push "START"
#	Maybe converts the value of source to decimal digits and copies it to DST
	incrementing DST after each digit. A way to print. Managed better by 'r' as radix?
+	set operation to add. a+b adds b to a. a:b+5 sets a to b then adds 5. 
	if there is no SRC, the NUM is used as the SRC. a+1 increments a.
	maybe if last op was +, load 1 into NUM. a++ increments a.
-	set operation to subtract
&	set operation to bitwise AND. a-&b ANDs a with NOT b. a&-b subtracts b from a (& is ignored)
|	set operation to bitwise OR
=	set compare type to equal
<	set compare type to less than
>	set compare type to greater than
{	Less than or equal (ASCII value of '<' plus '=' less 63)
}	Greater than or equal (ASCII value of '>' plus '=' less 63)
~	Not. Toggle true/false flag. Use with greater less and equal. 
		; e.g. a<b~ will set the true flag if a is greater than or equal to b.
		; >~ is less than or equal too. <~ is greater than or equal too. =~ is not equal
	perhaps change to ` (single back tick) (ASCII value of '!' plus '=' less 63)
?	if. Skip to the next line if the comparison fails (not TRUE)& keep skipping indented lines.
!	(or \?) else. Skip to the next line if the comparison succeeded & keep skipping indented lines. 
(	parms. Prep for a function call by pushing state.
, push accumulated NUM as an argument to the stack and increment SP. Clear NUM for the next 
)	call. Push final argument, push PC, and call the function pointed to by DST by DST to PC.
[	Start loop
]	End loop if true flag is not set.
.	return. Cleanup stack, restore PC.
A	set Port pin in SRC to read analog values in e.g. a:2A.
D Device. Complex device like Stepper Motor, I2C, SPI, etc.. See below for details.
J	(Jump) move NUM lines ?
I	(In) set the Port or Port pin in SRC to an Input. E.g. a:7I reads pin 7 ADC into a
H	(High) set the Pin in DST to high. e.g. 1H sets pin 1 high.
L	(Low) set the Pin in DST to low.
	When the pin is an input, H and L set or clear TRUE based on the pins value.
P	(PWM) set Pin in DST to output PWM in SRC. e.g. 2P100
R	(RC Servo) set Port pin in DST to drive RC servo to postion in SRC. e.g. 1R90
T	(Terminal) set SRC or DST to the Serial port. 0x89
U	(Up) set Pin in DST to inputs will internal pull-up
W	wait. Delay for DST u seconds between IO commands. Clears DST. e.g. 100DP0HLHL

Devices: These are the mnemonic character literals passed to the D (Device) 
function to allocate and configure hardware peripherals.:
'A' : Analog Input / ADC (Type, Pin) or use A.
'D' : Dynamixel Servo. Arguments: (Type, ID, BaudRate).
'i' : I2C Bus. Arguments: (Type, SclPin, SdaPin).
'I' : Digital Input (Type, Pin, Pull) or use I, or U.
'O' : Digital Output (Type, Pin, Value) or use H, L. 
'P' : PWM Output (Type, Pin, Value) or use P.
'Q' : Quadrature Encoder. Arguments: (Type, PinA, PinB).
'R' : RC Servo (Type, Pin, Angle) or use S.
's' : SPI Bus. Arguments: (Type, MisoPin, MosiPin, SckPin).
'S' : Stepper Motor. Arguments: (Type, StepPin, DirPin, MaxVel, Accel).
'U' : UART / Serial. Arguments: (Type, TxPin, RxPin, BaudRate).


Unused (for now)
$	
%	printf 
;	chains commands together, like a newline
^	power? 
_	label? subelement?

*/
#define BAUD 115200
//#define BAUD 57600 //Default Dynamixel baudrate
//The same baud rate should be used for both the Arduino and the Dynamixel. e.g. if you want to talk to the
//Arduino at 115200, then you need to re-program the Dynamixel to be at 115200. Of course, if you don't use
//the Dynamixels, you can set this to whatever you like. 

#define DYNAMIXEL_SUPPORT
//Note: If enabled, the Dynamixel Arduino shield is required and the standard Arduino UNO serial port is NOT functional! 
//You must install the USB to Serial interface on the Dynamixel Shield and use that for communication. RTFM

#define STEPPER_SUPPORT

//#define EOT
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

// --- CORE ABC VIRTUAL MACHINE STATE ---
// The Tagged Union structure for dynamically typed targets!
#define TYPE_NUM 0
#define TYPE_REG 1
#define TYPE_PIN 2
#define TYPE_DEV 3
#define TYPE_FUNC 4

union ABCState {
  long raw;              // Allows a full 32-bit push/pop to the stack
  struct {
    int16_t value;       // Payload: register index, pin number, device ID
    uint8_t type;        // TYPE_NUM, TYPE_REG, TYPE_PIN, TYPE_DEV
    uint8_t arg_count;   // Free byte! Useful for storing arg counts later
  } meta;
};

ABCState vars[26];   // Upgrade Registers 'a' through 'z' to Typed States!
#define radix vars['r'-'a'].meta.value // Alias 'r' to radix payload value
#define sp    vars['s'-'a'].meta.value // Alias 's' to stack pointer payload value
#define pc    vars['p'-'a'].meta.value // Alias 'p' to program counter payload value

long stack[256];            // The actual memory stack
int frame_pointer = 0;      // Points to start of arguments in the current call
int current_arg_count = 0;  // How many args in the current scope
int call_depth = 0;         // Are we inside a subroutine?

bool in_char_literal = false; // For parsing 'm'
long num = 0;                 // Numeric accumulator

ABCState dst; // Current Destination
ABCState src; // Current Source

char op = 0;          // Current Operation
bool src_dst = false; // False = looking for DST, True = looking for SRC
bool true_flag = true; // For conditionals (?)
bool skip_line = false; 

// --- EXECUTION & LOOP CONTROL ---
const char* pc_ptr = nullptr; // Tracks our current position in the bytecode string
const char* loop_stack[8];    // Supports nested loops up to 8 levels deep
int loop_depth = 0;           // Current loop nesting level

// --- HARDWARE SHADOWS ---
long pin_shadow[30];   // Stores digital (0/1) or PWM values
char pin_type[30];     // Mnemonic tags: 
long mock_servos[30];  // Stores servo angles

// --- COMPLEX DEVICE MANAGEMENT ---
#define MAX_DEVICES 10

struct DeviceAllocation {
  char type;         // e.g. 'O', 'I', 'P', 'A', 'R', etc...
  uint8_t pin1;      // Primary Pin (e.g., Output pin, Step, Tx, SCL)
  uint8_t pin2;      // Secondary Pin (e.g., Dir, Rx, SDA)
  long shadow_value; // Tracks current duty cycle, servo angle, or pin state
};

DeviceAllocation allocated_devices[MAX_DEVICES];
int next_device_index = 1; // Start at 1 so Handle ID 0 remains a standard null/number

void delayus(unsigned long us) {
  if (us>1000) { //can't delayMicroseconds() more than 16838
    delay(us/1000);
    us=us % 1000;
    }
  delayMicroseconds(us);
  }

#ifdef DYNAMIXEL_SUPPORT
void rebootServo(int id, int mode) { //setup servo id number into mode.
  dxl.torqueOff(id);
  if (!mode) mode = SERVO_MODE;
  DEBUG_SERIAL.print("{\"Servo\": ");
  DEBUG_SERIAL.print(id);
  if (dxl.setOperatingMode(id, mode)) {
    DEBUG_SERIAL.print(", \"Mode\": ");
    DEBUG_SERIAL.print(mode);
    }
  else {
    DEBUG_SERIAL.println(", \"Status\": \"Error\"}");
    return;
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
  DEBUG_SERIAL.println("}");
#ifdef EOT
  DEBUG_SERIAL.write(04); //EOT
#endif
  }
#endif

// --- INTERPRETER STUB ---

long* getVarPtr(int regIndex) {
  // Shadow parameters if inside a function call
  if (call_depth > 0 && regIndex < current_arg_count) {
    return &stack[frame_pointer + regIndex];
  }
  return (long*)&vars[regIndex].meta.value;
}

void doop() {
  if (op == 0) return; // No operation to perform
  long* target = nullptr;
  
  // Resolve the target pointer if the destination is a standard memory register
  if (dst.meta.type == TYPE_REG) {
    target = getVarPtr(dst.meta.value);
  }

  switch (op) {
    case ':':
      if (dst.meta.type == TYPE_PIN) {
        if (pin_type[dst.meta.value] == 'R') {
          mock_servos[dst.meta.value] = num;
          // REAL HARDWARE: servo[dst.meta.value].write(num);
        } else {
          pin_shadow[dst.meta.value] = num; 
          pinMode(dst.meta.value, OUTPUT);
          if (num > 1) {
            pin_type[dst.meta.value] = 'P'; 
            analogWrite(dst.meta.value, num);
          } else {
            pin_type[dst.meta.value] = 'O'; 
            digitalWrite(dst.meta.value, num);
          }
        }
      } else if (dst.meta.type == TYPE_REG) {
        // SMART WRITE: Check if the variable space already holds an active peripheral handle
        if (vars[dst.meta.value].meta.type == TYPE_DEV) {
          int handle = vars[dst.meta.value].meta.value;
          char d_type = allocated_devices[handle].type;
          uint8_t target_pin = allocated_devices[handle].pin1;

          allocated_devices[handle].shadow_value = num; // Keep internal tracker aligned

          if (d_type == 'O') {
            digitalWrite(target_pin, num);
          } else if (d_type == 'P') {
            analogWrite(target_pin, num);
          } else if (d_type == 'R') {
            mock_servos[target_pin] = num;
            // REAL HARDWARE: servo[target_pin].write(num);
          }
        } else {
          // No device attached. If the incoming source is an allocated device handle,
          // migrate the handle configuration and its type properties directly into this register!
          if (src.meta.type == TYPE_DEV) {
            vars[dst.meta.value].meta.type = TYPE_DEV;
            vars[dst.meta.value].meta.value = src.meta.value;
          } else {
            *target = num; // Standard variable literal assignment
          }
        }
      }
      break;
      
    case '+': if (target) *target += num; break;
    case '-': if (target) *target -= num; break;
    case '&': if (target) *target &= num; break;
    case '|': if (target) *target |= num; break;
    case '=': if (target) true_flag = (*target == num); break;
    case '<': if (target) true_flag = (*target < num); break;
    case '>': if (target) true_flag = (*target > num); break;
    case '{': if (target) true_flag = (*target <= num); break; 
    case '}': if (target) true_flag = (*target >= num); break; 
    
    case 'P': // Immediate inline PWM Operation modifier (e.g. 2P100)
      if (dst.meta.type == TYPE_PIN) {
        pin_shadow[dst.meta.value] = num;
        pin_type[dst.meta.value] = 'P';
        pinMode(dst.meta.value, OUTPUT);
        analogWrite(dst.meta.value, num);
      }
      break;

    case 'R': // Immediate inline Servo Operation modifier (e.g. 1R90)
      if (dst.meta.type == TYPE_PIN) {
        mock_servos[dst.meta.value] = num;
        pin_type[dst.meta.value] = 'R';
        // REAL HARDWARE: servo[dst.meta.value].write(num);
      }
      break;
  }
  
  // Clear source and number, but keep destination and operation
  num = 0;
  src.raw = 0; // The union zero-init clears type, value, and args simultaneously!
}

void processChar(char c) {
  if (skip_line) {
    if (c == '\n' || c == '\r') {
      skip_line = false; // Reset at end of line
    }
    return;
  }

  // Single Quote Character Parsing
  if (c == '\'') {
    in_char_literal = !in_char_literal; // Toggle mode
    return;
  }
  if (in_char_literal) {
    num = c; // Grab the ASCII decimal value
    return;
  }

  // Loop Operators
  if (c == '[') {
    if (loop_depth < 8) {
      loop_stack[loop_depth++] = pc_ptr; // Save the address of the '[' character
    }
    return;
  }

  if (c == ']') {
    doop(); // Execute any pending operations before evaluating the loop exit
    if (true_flag) {
      if (loop_depth > 0) {
        // Jump back! (The evaluateABC loop will increment this by 1, putting us right after '[')
        pc_ptr = loop_stack[loop_depth - 1]; 
      }
    } else {
      if (loop_depth > 0) {
        loop_depth--; // Pop the loop stack and exit
      }
    }
    num = 0; op = 0; src_dst = false; dst.raw = 0; src.raw = 0;
    return;
  }

  // Stack Operators: Call Start
  if (c == '(') {
    // Pack current VM execution state to the stack
    long state_flags = (op << 8) | (src_dst ? 1 : 0);
    stack[sp++] = dst.raw;
    stack[sp++] = src.raw;
    stack[sp++] = state_flags;
    current_arg_count = 0;
    return;
  }

  // Stack Operators: Push Arg
  if (c == ',') {
    stack[sp++] = num;
    current_arg_count++;
    num = 0; op = 0; src_dst = false; 
    return;
  }

  // Stack Operators: Execute Call
  if (c == ')') {
    stack[sp++] = num;
    current_arg_count++;

    long ret_val = 0; 

    // COMPLEX DEVICE SYSTEM CALL MANAGER ('D')
    if (src.meta.type == TYPE_FUNC && src.meta.value == 'D') {
     // Find where our arguments begin on the stack frame
      int arg_start = sp - current_arg_count;
      char dev_type = (char)stack[arg_start]; // First argument pushed is our Type literal

      if (next_device_index < MAX_DEVICES) {
        allocated_devices[next_device_index].type = dev_type;
        
        switch (dev_type) {
          case 'O': // Digital Output (Type, Pin, DefaultValue)
            allocated_devices[next_device_index].pin1 = stack[arg_start + 1];
            allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
            pinMode(allocated_devices[next_device_index].pin1, OUTPUT);
            digitalWrite(allocated_devices[next_device_index].pin1, allocated_devices[next_device_index].shadow_value);
            break;

          case 'I': // Digital Input (Type, Pin, PullupMode)
            allocated_devices[next_device_index].pin1 = stack[arg_start + 1];
            // If third argument is 1, configure as pullup, otherwise standard input
            if (stack[arg_start + 2] == 1) {
              pinMode(allocated_devices[next_device_index].pin1, INPUT_PULLUP);
            } else {
              pinMode(allocated_devices[next_device_index].pin1, INPUT);
            }
            break;

          case 'P': // PWM Output (Type, Pin, DefaultDuty)
            allocated_devices[next_device_index].pin1 = stack[arg_start + 1];
            allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
            pinMode(allocated_devices[next_device_index].pin1, OUTPUT);
            analogWrite(allocated_devices[next_device_index].pin1, allocated_devices[next_device_index].shadow_value);
            break;

          case 'A': // Analog Input (Type, Pin)
            allocated_devices[next_device_index].pin1 = stack[arg_start + 1];
            pinMode(allocated_devices[next_device_index].pin1, INPUT);
            break;

          case 'R': // RC Servo (Type, Pin, DefaultAngle)
            allocated_devices[next_device_index].pin1 = stack[arg_start + 1];
            allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
            // REAL HARDWARE: servo[pin1].attach(pin1); servo[pin1].write(shadow_value);
            mock_servos[allocated_devices[next_device_index].pin1] = allocated_devices[next_device_index].shadow_value;
            break;
        }

        ret_val = next_device_index; // Pass back allocated index as our reference tag
        next_device_index++;
      }
    }

    sp -= current_arg_count;
    current_arg_count = 0; 

    // Restore the VM state exactly as we left it
    long state_flags = stack[--sp];
    src.raw = stack[--sp];
    dst.raw = stack[--sp];

    src_dst = (state_flags & 1) != 0;
    op = (state_flags >> 8) & 0xFF;

    // Package our returned index directly as an explicit TYPE_DEV token!
    if (ret_val > 0) {
      num = 0;
      src.meta.type = TYPE_DEV;
      src.meta.value = ret_val;
    } else {
      num = ret_val; 
    }
    return;
  }

  // Accumulate Numbers (Based on 'r' register radix)
  if (c >= '0' && c <= '9') {
    num *= radix;
    num += (c - '0');
    return;
  }
  if (radix == 16 && (c >= 'a' && c <= 'f')) {
    num *= radix;
    num += (c - 'a' + 10);
    return;
  }

// Identify Variables / Registers (a-z)
  if (c >= 'a' && c <= 'z') {
    int regIndex = c - 'a';
    if (!src_dst) { // Looking for destination
      dst.meta.type = TYPE_REG;
      dst.meta.value = regIndex;
      src_dst = true; 
    } else {        // Looking for source
      src.meta.type = TYPE_REG;
      src.meta.value = regIndex;
      
      // SMART READ: If this register holds an active Device Handle, execute a live peripheral sweep!
      if (vars[regIndex].meta.type == TYPE_DEV) {
        int handle = vars[regIndex].meta.value;
        char d_type = allocated_devices[handle].type;
        uint8_t target_pin = allocated_devices[handle].pin1;

        if (d_type == 'I') {
          num = digitalRead(target_pin);
        } else if (d_type == 'A') {
          num = analogRead(target_pin);
        } else if (d_type == 'R') {
          num = mock_servos[target_pin]; // Return active tracking state
        } else {
          num = allocated_devices[handle].shadow_value; // Fallback to recorded memory state
        }
      } else {
        num = *getVarPtr(regIndex); // Standard variable register readout
      }
    }
    return;
  }

  // Handle Built-In Functions (e.g., 'D' for Device Initialization)
  if (c == 'D') {
    if (!src_dst) {
      dst.meta.type = TYPE_FUNC;
      dst.meta.value = c;
      src_dst = true;
    } else {
      src.meta.type = TYPE_FUNC;
      src.meta.value = c;
    }
    return;
  }

  // Destination Pin Assignment / Read Shadow Modifier ('P' or 'R')
  if (c == 'P' || c == 'R') {
    if (!src_dst) { 
      dst.meta.type = TYPE_PIN;
      dst.meta.value = num;
      if (c == 'R') pin_type[num] = 'R'; // Pre-mark it as a servo
      src_dst = true; 
      num = 0; // Reset accumulator ONLY for the destination
    } else {        
      src.meta.type = TYPE_PIN;
      src.meta.value = num;
      num = (c == 'R') ? mock_servos[num] : pin_shadow[num]; 
    }
    return;
  }

  // Immediate Hardware Output Commands (H, L)
  if (c == 'H' || c == 'L') {
    pin_shadow[num] = (c == 'H') ? 1 : 0; 
    pin_type[num] = 'O';
    pinMode(num, OUTPUT);
    digitalWrite(num, (c == 'H') ? HIGH : LOW);
    num = 0; op = 0; src_dst = false; dst.raw = 0;
    return;
  }

  // Active Read Modifiers (I, U, A) - Read hardware, retain value in accumulator for assignment!
  if (c == 'I') { // Digital Input
    pin_type[num] = 'I';
    pinMode(num, INPUT);
    num = digitalRead(num); 
    return;
  }

  if (c == 'U') { // Input Pull-Up
    pin_type[num] = 'u';
    pinMode(num, INPUT_PULLUP);
    num = digitalRead(num); 
    return;
  }

  if (c == 'A') { // Analog Input 
    pin_type[num] = 'A';
    pinMode(num, INPUT);
    num = analogRead(num); 
    return;
  }

  // Toggle true_flag (Not operator)
  if (c == '~') {
    doop();
    true_flag = !true_flag;
    return;
  }

  // Handle Conditionals
  if (c == '?') {
    doop(); // Evaluate the condition first!
    if (!true_flag) skip_line = true;
    src_dst = false; op = 0; //reset state for next line
    return;
  }

  // Handle Inverted Conditionals (Else/If Not)
  if (c == '!') {
    doop(); 
    if (true_flag) skip_line = true;
    src_dst = false; op = 0; 
    return;
  }

  // Immediate Hardware/State Operations (H, L, I, U, W)
  if (c == 'W') { // Wait (Delay in microseconds)
    // long ms = num/1000;
    // long us = num%1000;
    // while(ms > 0) {
    //   int analogValue = analogRead(26);
    //   DEBUG_SERIAL.println(analogValue);
    //   delay(100);
    //   ms -= 100;
    // }
    delayus(num);
    num = 0; op = 0; src_dst = false; dst.raw = 0;
    return;
  }

  // End of Line / Execution Trigger (Now supports ';' chaining)
  if (c == '\n' || c == '\r' || c == ';') {
    doop(); 
    num = 0; op = 0; src_dst = false; true_flag = false;
    dst.raw = 0; src.raw = 0; 
    return;
  }

  // If it's none of the above, it's likely a standard operator
  if (c != ' ' && c != '\t') { 
    doop(); 
    op = c;
  }
}

void evaluateABC(const char* commands) {
  pc_ptr = commands;
  while (*pc_ptr) {
    processChar(*pc_ptr);
    pc_ptr++; // Advance to the next character
  }
}

// --- TEST HARNESS FRAMEWORK ---
int testCount = 0;
int passCount = 0;


void resetTestState() {
  for(int i=0; i<26; i++) {
    vars[i].raw = 0;
    vars[i].meta.type = TYPE_REG; // Set type metadata
  }
  for(int i=0; i<30; i++) {
    pin_shadow[i] = 0;
    pin_type[i] = 0;
    mock_servos[i] = 0;
  }
  radix = 10; 
  num = 0; 
  dst.raw = 0; 
  src.raw = 0; 
  op = 0; 
  src_dst = false; 
  true_flag = true; 
  skip_line = false;
  next_device_index = 1;
  loop_depth = 0; 
  pc_ptr = nullptr;
}

void printCodeIndented(const char* code) {
  DEBUG_SERIAL.print("       Code: ");
  while (*code) {
    if (*code == '\n') {
      DEBUG_SERIAL.print("\r\n");
      if (*(code + 1) != '\0') {
        DEBUG_SERIAL.print("             "); // Indent the next line
      }
    } else if (*code != '\r') {
      DEBUG_SERIAL.print(*code);
    }
    code++;
  }
  if (*(code - 1) != '\n') DEBUG_SERIAL.print("\r\n");
}

void runTest(const char* testName, const char* code, char checkReg, long expectedValue) {
  testCount++;
  
  resetTestState();

  // 2. Run the bytecode
  evaluateABC(code);

  // 3. Assert the result
  long actualValue = vars[checkReg - 'a'].meta.value;
  if (actualValue == expectedValue) {
    DEBUG_SERIAL.printf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    DEBUG_SERIAL.printf("       Expected register '%c' to be %ld, but got %ld\r\n", checkReg, expectedValue, actualValue);
  }
}

void runAnalogTest(const char* testName, const char* code, char checkReg, long expectedValue, long tolerance) {
  testCount++;
  resetTestState();
  evaluateABC(code);
  long actualValue = vars[checkReg - 'a'].meta.value;
  if (abs(actualValue - expectedValue) <= tolerance) {
    DEBUG_SERIAL.printf("[PASS] %s (Got %ld, expected ~%ld+/-±%ld)\r\n"
      , testName, actualValue, expectedValue, tolerance
    );
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    DEBUG_SERIAL.printf("       Expected register '%c' to be approx %ld (±%ld), but got %ld\r\n"
      , checkReg, expectedValue, tolerance, actualValue
    );
  }
}

void runHardwareTest(const char* testName, const char* code, int checkPin, long expectedValue, bool isServo = false) {
  testCount++;
  
  // Reset VM and Mock Hardware
  resetTestState();

  evaluateABC(code);

  long actualValue = isServo ? mock_servos[checkPin] : pin_shadow[checkPin];
  
  if (actualValue == expectedValue) {
    Serial.printf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    Serial.printf("[FAIL] %s\r\n       Code: %s\r       Expected Pin %d to be %ld, but got %ld\r\n"
      ,testName, code, checkPin, expectedValue, actualValue
    );
  }
}

void setup() {
  DEBUG_SERIAL.begin(BAUD);
  delay(2000); 

  DEBUG_SERIAL.println("\r\n--- ABC Language Test Harness ---");

  // TEST 1: Basic Assignment (Destination, Operation, Number)
  runTest("Basic Numeric Assignment", "a:5\n", 'a', 5);

  // TEST 2: Register to Register Assignment
  runTest("Register Copy", "a:9\nb:a\n", 'b', 9);

  // TEST 3: Math Addition
  runTest("Simple Addition", "a:5\na:a+2\n", 'a', 7);

  // TEST 4: Chained Operations
  runTest("Implicit Destination Math", "b:2\na:b+3\n", 'a', 5); 

  // TEST 5: Conditionals (True)
  runTest("Conditional True", "a:1\na=1?b:9\n", 'b', 9);

  // TEST 6: Conditionals (False)
  runTest("Conditional False", "a:0\na=1?b:9\n", 'b', 0);

  // TEST 7: Digital High Output
  runHardwareTest("Digital Pin High", "13H\n", 13, 1);

  // TEST 8: Digital Low Output
  runHardwareTest("Digital Pin Low", "9L\n", 9, 0);

  // TEST 9: Analog/PWM Output
  runHardwareTest("Analog PWM Output", "5P:128\n", 5, 128);

  // TEST 10: Servo Output
  runHardwareTest("Servo Position", "2R:90\n", 2, 90, true);

  // TEST 11: Radix Switching (Base 16)
  // Set radix to 16, then load hex "10" into 'g'. We can't use 'a' because a-f  
  // are reserved for hex digits. Expected decimal value: 16
  runTest("Radix Hex", "r:16\ng:10\n", 'g', 16);

  // TEST 12: Radix Switching (Base 2)
  // Set radix to 2, then load binary "101" into 'b'. Expected decimal value: 5
  runTest("Radix Binary", "r:2\nb:101\n", 'b', 5);

  // TEST 13: Single Quote ASCII parsing
  // Assign the ASCII value of 'm' (109) to 'a'
  runTest("Single Quote ASCII", "a:'m'\n", 'a', 109);

  // TEST 14: Comma Pushing & Stack Pointer Manipulation
  // Push 10, 20 to stack. The 's' register (SP) should increment by 2.
  runTest("Stack Pointer Update", "10, 20,\n", 's', 2);

  // TEST 15: Mock Hardware Device Handle
  // D is the hardware init function. 'M' is type motor. It returns handle 99 to 'a'.
  runTest("Hardware Function Call", "a:D('M', 3, 4)\n", 'a', 1);

  // TEST 16: Variable Parameter Shadowing
  // We manually spoof entering a subroutine with 2 arguments already on the stack.
  // 'a' should map to stack[0], and 'b' should map to stack[1]. 'c' maps to global.
  resetTestState(); // Use the formal reset to wipe type metadata!
  testCount++;
  stack[0] = 77; // Spoof Arg 1
  stack[1] = 88; // Spoof Arg 2
  sp = 2;
  frame_pointer = 0;
  current_arg_count = 2;
  call_depth = 1; // SPOOF WE ARE INSIDE A FUNCTION
  
  // Try to copy 'a' to 'c'. If shadowing works, 'c' becomes 77, not 0!
  evaluateABC("c:a\n"); 
  if (vars['c'-'a'].meta.value == 77) {
    DEBUG_SERIAL.printf("[PASS] Variable Parameter Shadowing\r\n");
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] Variable Parameter Shadowing\r\n");
  }
  call_depth = 0; // reset

  // TEST 17: Command Chaining with Semicolon
  // Assign 1 to a, 2 to b, then add them into c, all on one line.
  runTest("Semicolon Chaining", "a:1; b:2; c:a+b\n", 'c', 3);

  // TEST 18: Do-While Loop Mechanics
  // Initialize a to 0. Inside the loop, increment a by 1. Loop while a < 5.
  runTest("Loop Incrementing", "a:0\n[\na:a+1\na<5]\n", 'a', 5);

// --- HARDWARE IN THE LOOP (HIL) TESTS ---
// Ensure your Wokwi layout connects GP2 to GP3, and contains a simulated RC network from GP22 into GP26 (A0)
// via a WOKWI filter to average the "analog" PWM output and feed the analog input. 
// https://wokwi.com/projects/409325290405496833
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  delay(10); 
  if (digitalRead(3) == HIGH) {
    DEBUG_SERIAL.println("[PASS] Native C++ Wiring Check (GP2 -> GP3 connected)");
  } else {
    DEBUG_SERIAL.println("[FAIL] Simulator not setup! Please check diagram.json.");
  }
  digitalWrite(2, LOW);

  // 1. Set Pin 3 to Input
    evaluateABC("3P I\n");
  // 2H fires output. 1000W yields. 3I captures digital input state directly into variable 'a'.
  runTest("HIL: Active DigIn High Read", "2H\n1000 W\na:3I\n", 'a', 1);
  runTest("HIL: Active DigIn Low Read", "2L\n1000 W\nb:3I\n", 'b', 0);
  
  // 2. Analog Simulation Filter Catch
  // Fire inline PWM 128 (50% scale) -> Sleep 50ms for RC settlement -> sample GP26 ADC straight to 'c'
  // 50% voltage on a 10-bit core (1023 max) yields approx 512. 1/4 is about 256.
  runAnalogTest("HIL: PWM to Analog Filter 1", "22P:127\n500000W\nc:26A\n", 'c', 512, 20);
  runAnalogTest("HIL: PWM to Analog Filter 2", "22P:63\n500000W\nc:26A\n", 'c', 256, 20);

  // --- ADVANCED PERIPHERAL REGISTER (HIL) TESTS ---

  // 1. Digital Output and Input Device Blocks
  // Setup 'o' on Pin 2 as High output, 'i' on Pin 3 as Input. Read 'i' into 'a'.
  runTest("Device: Digital I/O Handle Move", "o:D('O',2,1)\n1000 W\ni:D('I',3,0)\na:i\n", 'a', 1);

  // Toggle our output device handle low, and confirm the input device handle updates smoothly!
  runTest("Device: Digital Write Toggle", "o:D('O',2,1)\ni:D('I',3,0)\no:0\n1000 W\nb:i\n", 'b', 0);

  // 2. Analog/PWM Block Handle Testing
  // Setup 'o' as a PWM output on pin 22, and 'i' as an Analog ADC listener on pin 26
  runAnalogTest("Device: PWM/Analog Filter Integration", "o:D('P',22,127)\ni:D('A',26)\n500000 W\nc:i\n", 'c', 512, 20);

  // 3. RC Servo Configuration Stability
  // Set variable 'v' ('r' is radix) as an RC Servo on Pin 2 with a default tracking position of 90 degrees
  // Then use 'v:120' to alter position. The structure validates that our base handle remains uncorrupted.
  runHardwareTest("Device: RC Servo Target Stability", "v:D('R',2,90)\nv:120\n", 2, 120, true);

  DEBUG_SERIAL.printf("\r\n--- Test Run Complete: %d/%d Passed ---\r\n", passCount, testCount);
   
}

void loop() {
  // halt and look for input
  String input = DEBUG_SERIAL.readStringUntil('\n'); // Wait for user input 
  if (input.length() > 0) {
    DEBUG_SERIAL.printf("Evaluating ABC code:\n%s\n", input.c_str());
    evaluateABC(input.c_str());
    DEBUG_SERIAL.println("Done evaluating.\n");
  }
  delay(1); // Small delay to avoid busy looping
}