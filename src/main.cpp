#include <Arduino.h>
#define TEST // Comment this line out to compile for REAL hardware!

#ifndef TEST
#include <EEPROM.h>
#include <Servo.h>
Servo physical_servos[NUM_DIGITAL_PINS]; 
#endif

/*
Goals 
- get close to a high level language, w/ understandable syntax, without including a compiler, 
- using the minimum resources possible to interpret the bytecodes. 
- The main pattern is: Destination, [Operation, Source ...], e.g. a=b+c-d. 
- The regular keywords in a language are single characters, making use of punctuation

0-9 (and lowercase a and on when radix is over 10) these accumulate base-10 digits into num 
a-z Variables/Registers (SRC or DST). Some registers have special meanings:
   p Program Counter aka PC.
   q Queue length (RX ring buffer). Read/Write to manage incoming stream.
   r Radix (Default 10).
   s Stack Pointer aka SP.
   t Terminal Device (Default serial output/input).
  Might not be contiguous in the array. e.g. while a is 0. b could be register 4, c 8, etc..
:	Copy operation, e.g. "define" a:5 sets register 0 to 5. a:b copies the value of b to register 0.
	NUM may be added to the register address first. 3a could be the third byte after the start of a. 
@	index. Replace SRC or DST with the value at that address
	 and clear op. This sets the stage for another op and SRC.
	e.g. b@a sets the DST to the address of b plus the value of a.
	If the SRC is a port or port pin, read that value in. 
' (Single Quote) ASCII literal. Next character is its numeric decimal value ('A' is 65).
"	(Quote) Text. Each following char is copied to the DST until the ending quote.
	If the DST is a variable, the chars are actually copied into memory and the var is
	set to the starting address of the string in memory.
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
// The Tagged Union structure for dynamically typed targets
// This is what I've been missing all these years.
#define TYPE_NUM 0
#define TYPE_REG 1
#define TYPE_PIN 2
#define TYPE_DEV 3
#define TYPE_FUNC 4
#define TYPE_CODE 5

union ABCState {
  long raw;              // Allows a full 32-bit push/pop to the stack
  struct {
    int16_t value;       // Payload: register index, pin number, device ID
    uint8_t type;        // TYPE_NUM, TYPE_REG, TYPE_PIN, TYPE_DEV
    uint8_t arg_count;   // Free byte! Useful for storing arg counts later
  } meta;
};

ABCState vars[26];   // Upgrade Registers 'a' through 'z' to Typed States
#define radix vars['r'-'a'].meta.value // Alias 'r' to radix payload value
#define sp    vars['s'-'a'].meta.value // Alias 's' to stack pointer payload value
#define pc    vars['p'-'a'].meta.value // Alias 'p' to program counter payload value

long stack[256];            // The actual memory stack
int frame_pointer = 0;      // Points to start of arguments in the current call
int current_arg_count = 0;  // How many args in the current scope
int call_depth = 0;         // Are we inside a subroutine?

char code_ram[1024]; // Volatile dictionary for user functions
int code_ptr = 0;
bool quote_pending = false; // For detecting "" escapes
bool in_string_literal = false;
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
long pin_shadow[NUM_DIGITAL_PINS];   // Stores digital (0/1) or PWM values
char pin_type[NUM_DIGITAL_PINS];     // Mnemonic tags: 

// --- CUSTOM RX BUFFER FOR STRING MATCHING & REPL ---
char rx_buffer[64];
int rx_head = 0;
int rx_tail = 0;
bool in_repl = false;

// --- MOCK BUFFERS FOR TESTING ---
#ifdef TEST
char mock_out_buffer[256];
int mock_out_ptr = 0;
char mock_eeprom[1024]; // Simulates Arduino EEPROM / Flash
#endif
int eeprom_ptr = 0;
long mock_servos[NUM_DIGITAL_PINS];  // Stores servo angles

// --- COMPLEX DEVICE MANAGEMENT ---
#define MAX_DEVICES 10

struct DeviceAllocation {
  char type;         // e.g. 'O', 'I', 'P', 'A', 'R', etc...
  uint8_t pinA;      // Primary Pin (e.g., Output pin, Step, Tx, SCL)
  uint8_t pinB;      // Secondary Pin (e.g., Dir, Rx, SDA)
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

bool matchStream(const char* target_str) {
  int peek_tail = rx_tail; // Create a temporary cursor to peek ahead
  int i = 0;

  while (target_str[i] != '\0') {
    // 1. Block character-by-character until we have something to evaluate
    while (peek_tail == rx_head) {
      if (DEBUG_SERIAL.available()) {
        rx_buffer[rx_head % 64] = DEBUG_SERIAL.read();
        rx_head++;
      }
    }

    // 2. Evaluate the character!
    if (rx_buffer[peek_tail % 64] != target_str[i]) {
      return false; // MISMATCH! We return false immediately and leave rx_tail completely untouched!
    }

    // 3. It matched! Move our peek cursor forward and check the next character.
    peek_tail++;
    i++;
  }

  // 4. The entire string matched! Consume the characters permanently.
  rx_tail = peek_tail; 
  return true;
}

void devWrite(int handle, char c) {
  if (allocated_devices[handle].type == 'T') {
#ifdef TEST
    if (mock_out_ptr < 255) {
      mock_out_buffer[mock_out_ptr++] = c;
      mock_out_buffer[mock_out_ptr] = '\0';
    }
    if (in_repl) DEBUG_SERIAL.print(c);
#else
    DEBUG_SERIAL.print(c); // Route Terminal directly to the real serial console!
#endif
  } else if (allocated_devices[handle].type == 'U') { 
    // UART handler placeholder
  } else if (allocated_devices[handle].type == 'F') { 
#ifdef TEST
    if (eeprom_ptr < 1023) {
      mock_eeprom[eeprom_ptr++] = c;
      mock_eeprom[eeprom_ptr] = '\0';
    }
#else
    if (eeprom_ptr < 1023) {
      EEPROM.write(eeprom_ptr++, c);
      // Note: On ESP8266/ESP32/RP2040 you must call EEPROM.commit() after writing a full string.
    }
#endif
  }
}

long allocateDevice(char dev_type, int arg_start) {
  if (next_device_index >= MAX_DEVICES) return 0;
  
  allocated_devices[next_device_index].type = dev_type;
  switch (dev_type) {
    case 'A':
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      pinMode(allocated_devices[next_device_index].pinA, INPUT);
      break;
    case 'F':
      eeprom_ptr = 0; 
#ifdef TEST
      mock_eeprom[0] = '\0';
#endif
      break;
    case 'I':
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      if (stack[arg_start + 2] == 1) pinMode(allocated_devices[next_device_index].pinA, INPUT_PULLUP);
      else pinMode(allocated_devices[next_device_index].pinA, INPUT);
      break;
    case 'O':
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
      pinMode(allocated_devices[next_device_index].pinA, OUTPUT);
      digitalWrite(allocated_devices[next_device_index].pinA, allocated_devices[next_device_index].shadow_value);
      break;
    case 'P':
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
      pinMode(allocated_devices[next_device_index].pinA, OUTPUT);
      analogWrite(allocated_devices[next_device_index].pinA, allocated_devices[next_device_index].shadow_value);
      break;
case 'R':
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
      mock_servos[allocated_devices[next_device_index].pinA] = allocated_devices[next_device_index].shadow_value;
#ifndef TEST
      physical_servos[allocated_devices[next_device_index].pinA].attach(allocated_devices[next_device_index].pinA);
      physical_servos[allocated_devices[next_device_index].pinA].write(allocated_devices[next_device_index].shadow_value);
#endif
      break;
    case 'T': 
      break;
  }
  return next_device_index++;
}

void executeReturn() {
  if (call_depth <= 0) return;
  
  long ret_val = num;
  if (dst.meta.type == TYPE_REG) ret_val = *getVarPtr(dst.meta.value);
  
  pc_ptr = (const char*)stack[--sp]; // Pop Return Address
  
  // Restore previous frame
  sp = frame_pointer; 
  current_arg_count = stack[--sp];
  frame_pointer = stack[--sp];
  long state_flags = stack[--sp];
  src.raw = stack[--sp];
  dst.raw = stack[--sp];
  
  src_dst = (state_flags & 1) != 0;
  op = (state_flags >> 8) & 0xFF;
  
  num = ret_val; 
  src.raw = 0;   
  call_depth--;
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
      } else if (dst.meta.type == TYPE_DEV) {
        int handle = dst.meta.value;
        if (src.meta.type == TYPE_CODE) {
          // Stream string literal to the device
          const char* str = &code_ram[src.meta.value];
          while (*str) devWrite(handle, *str++);
        } else {
          // Standard hardware parameter write (e.g. Servo Angle)
          allocated_devices[handle].shadow_value = num;
          // ... [Hardware write logic like analogWrite] ...
        }
      } else if (dst.meta.type == TYPE_REG) {
        // SMART WRITE: Check if the variable space already holds an active peripheral handle
        if (vars[dst.meta.value].meta.type == TYPE_DEV) {
          int handle = vars[dst.meta.value].meta.value;
          char d_type = allocated_devices[handle].type;
          if (src.meta.type == TYPE_DEV) {
            // Device Handle Reassignment (e.g., overwriting an old device with a new D() call)
            vars[dst.meta.value].meta.type = src.meta.type;
            vars[dst.meta.value].meta.value = src.meta.value;
          } else if (src.meta.type == TYPE_CODE) {
            // Stream string literal directly to the mapped device!
            const char* str = &code_ram[src.meta.value];
            while (*str) devWrite(handle, *str++);
          } else {
            // Standard hardware parameter write (e.g., Output Pin, Servo Angle)
            allocated_devices[handle].shadow_value = num; // Keep internal tracker aligned
            uint8_t target_pin = allocated_devices[handle].pinA;
            if (d_type == 'O') {
              digitalWrite(target_pin, num);
            } else if (d_type == 'P') {
              analogWrite(target_pin, num);
            } else if (d_type == 'R') {
              mock_servos[target_pin] = num;
#ifndef TEST
              physical_servos[target_pin].write(num);
#endif
            }
          }
        } else {
          // No device attached. If the incoming source is an allocated device handle OR Code Handle,
          // migrate the handle configuration and its type properties directly into this register
          if (src.meta.type == TYPE_DEV || src.meta.type == TYPE_CODE) {
            vars[dst.meta.value].meta.type = src.meta.type;
            vars[dst.meta.value].meta.value = src.meta.value;
          } else {
            *target = num; // Standard variable literal assignment
          }
        }
      }
      break;

    case '%': // FORMATTING OPERATOR
      {
        int handle = -1;
        // Resolve handle if accessed via raw device OR via variable proxy
        if (dst.meta.type == TYPE_DEV) {
          handle = dst.meta.value;
        } else if (dst.meta.type == TYPE_REG && vars[dst.meta.value].meta.type == TYPE_DEV) {
          handle = vars[dst.meta.value].meta.value;
        }
        
        if (handle != -1) {
          if (src.meta.type == TYPE_CODE) {
            // Dump the function definition wrapped in quotes!
            devWrite(handle, '"');
            const char* str = &code_ram[src.meta.value];
            while (*str) devWrite(handle, *str++);
            devWrite(handle, '"');
          } else {
            // Format numeric accumulator into string using current radix
            char numStr[33];
            ltoa(num, numStr, radix);
            for (int i = 0; numStr[i] != '\0'; i++) {
              devWrite(handle, numStr[i]);
            }
          }
        }
      }
      break;

    case '+': if (target) *target += num; break;
    case '-': if (target) *target -= num; break;
    case '&': if (target) *target &= num; break;
    case '|': if (target) *target |= num; break;

    case '=': 
      {
        int handle = -1;
        // Resolve handle if accessed via raw device OR via variable proxy
        if (dst.meta.type == TYPE_DEV) {
          handle = dst.meta.value;
        } else if (dst.meta.type == TYPE_REG && vars[dst.meta.value].meta.type == TYPE_DEV) {
          handle = vars[dst.meta.value].meta.value;
        }
        
        // If we are comparing a Terminal/UART to a String!
        if (handle != -1 && src.meta.type == TYPE_CODE && 
           (allocated_devices[handle].type == 'T' || allocated_devices[handle].type == 'U')) {
          
          const char* str_to_match = &code_ram[src.meta.value];
          true_flag = matchStream(str_to_match);
          
        } else {
          // Standard numeric comparison
          if (target) true_flag = (*target == num); 
        }
      }
      break;

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
#ifndef TEST
        if (!physical_servos[dst.meta.value].attached()) {
          physical_servos[dst.meta.value].attach(dst.meta.value);
        }
        physical_servos[dst.meta.value].write(num);
#endif
      }
      break;
  }

  // SPECIAL REGISTER SYNC: If 'q' was modified, update the ring buffer tail!
  if (dst.meta.type == TYPE_REG && dst.meta.value == 'q' - 'a') {
    // Ensure we aren't shadowing 'q' as a function parameter
    if (!(call_depth > 0 && dst.meta.value < current_arg_count)) {
      int new_len = vars['q'-'a'].meta.value;
      if (new_len < 0) new_len = 0;
      
      int max_len = rx_head - rx_tail; // We can't magically create characters
      if (new_len > max_len) new_len = max_len;
      
      rx_tail = rx_head - new_len; // Slide the tail forward!
    }
  }

  // Clear source and number, but keep destination and operation
  num = 0;
  src.raw = 0; // The union zero-init clears type, value, and args simultaneously
}

void processChar(char c) {
  if (skip_line) {
    if (c == '\n' || c == '\r') {
      skip_line = false; // Reset at end of line
      return;
    } else if (c == '!') {
      skip_line = false; // Stop skipping so we can process the Else
      // Don't return, just fall through so the '!' handler can do its job.
    } else {
      return;
    }
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

  // Double Quote String Parsing & Escaping
  if (in_string_literal && c == '"') {
    if (quote_pending) {
      code_ram[code_ptr++] = '"'; // Escaped quote
      quote_pending = false;
    } else {
      quote_pending = true; // Wait to see if the next char is a quote or the end
    }
    return;
  } else if (in_string_literal) {
    if (quote_pending) {
      // It was a real ending quote. Terminate and exit string mode.
      in_string_literal = false;
      quote_pending = false;
      code_ram[code_ptr++] = '\0';
      
      // Package the string index as the Source
      src.meta.type = TYPE_CODE;
      src.meta.value = num;
      num = 0;
      // Do NOT return! Let the current character 'c' be processed normally (e.g., \n)
    } else {
      code_ram[code_ptr++] = c; // Record normal character
      return;
    }
  } else if (c == '"') {
    in_string_literal = true;
    quote_pending = false;
    num = code_ptr; // Save the starting index into the accumulator
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
    // Resolve standalone calls (e.g. "a()") where the function name was captured as a destination
    if (op == 0 && dst.meta.type == TYPE_REG && src.meta.type == 0) {
      src = dst;
      dst.raw = 0;
    }

    // Force function pointers to resolve from the global table
    // This ensures shadowed parameter names don't break function execution.
    if (src.meta.type == TYPE_REG) {
      int regIndex = src.meta.value;
      if (vars[regIndex].meta.type == TYPE_CODE) {
        src.meta.type = TYPE_CODE;
        src.meta.value = vars[regIndex].meta.value;
      } else if (vars[regIndex].meta.type == TYPE_DEV) {
        src.meta.type = TYPE_DEV;
        src.meta.value = vars[regIndex].meta.value;
      }
    }

    // Pack current VM execution state to the stack
    long state_flags = (op << 8) | (src_dst ? 1 : 0);
    stack[sp++] = dst.raw;
    stack[sp++] = src.raw;
    stack[sp++] = state_flags;
    stack[sp++] = frame_pointer;      // Save caller's frame
    stack[sp++] = current_arg_count;  // Save caller's arg count
    
    frame_pointer = sp; // New frame starts here
    src_dst = true;     // Ensure arguments are evaluated as sources
    current_arg_count = 0;
    return;
  }

  // Stack Operators: Push Arg
  if (c == ',') {
    stack[sp++] = num;
    current_arg_count++;
    num = 0; op = 0; src_dst = false;
    src_dst = true; // Ensure the NEXT argument is also a source
    return;
  }

  // Stack Operators: Execute Call
  if (c == ')') {
    stack[sp++] = num;
    current_arg_count++;

    // COMPLEX DEVICE SYSTEM CALL MANAGER ('D')
    if (src.meta.type == TYPE_FUNC && src.meta.value == 'D') {
     // Find where our arguments begin on the stack frame
      int arg_start = sp - current_arg_count;
      char dev_type = (char)stack[arg_start]; 
      
      long ret_val = allocateDevice(dev_type, arg_start);

      sp = frame_pointer; // Drop arguments
      current_arg_count = stack[--sp];
      frame_pointer = stack[--sp];
      long state_flags = stack[--sp];
      src.raw = stack[--sp];
      dst.raw = stack[--sp];

      src_dst = (state_flags & 1) != 0;
      op = (state_flags >> 8) & 0xFF;
      num = 0; 
      if (ret_val > 0) {
        src.meta.type = TYPE_DEV;
        src.meta.value = ret_val;
      } else {
        src.raw = 0;  // Clear the C++ function pointer from the source
      }
      return;
    }
    // USER-DEFINED BYTECODE FUNCTIONS
    else if (src.meta.type == TYPE_CODE) {
      stack[sp++] = (long)pc_ptr; // Push Return Address to the stack
      
      // OFFSET BY -1 because the evaluateABC loop will automatically do pc_ptr++ right after this
      pc_ptr = &code_ram[src.meta.value] - 1; 
      
      // Clean the active VM state so the function starts fresh
      op = 0;
      src_dst = false;
      dst.raw = 0;
      src.raw = 0;
      num = 0;
      call_depth++;
      return;
    }
  }

  // Return Operator
  if (c == '.') {
    doop(); // Execute any pending operations (like the final '+' in 'a+b')
    executeReturn();
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
      
      // If setting 'q' as destination, sync the live queue length into the register 
      // so implicit math (like q-1) has the correct base value to modify
      if (regIndex == 'q' - 'a' && !(call_depth > 0 && regIndex < current_arg_count)) {
        vars['q'-'a'].meta.value = rx_head - rx_tail;
      }
    } else {
      // Looking for source
      src.meta.type = TYPE_REG;
      src.meta.value = regIndex;
      
      // Parameter Shadowing:
      // If inside a function and index is a parameter, strictly use parameter value
      if (call_depth > 0 && regIndex < current_arg_count) {
        num = stack[frame_pointer + regIndex];
      } 
      // Special Register 'q' - Queue Length
      else if (regIndex == 'q' - 'a') {
        num = rx_head - rx_tail;
      }
      // Global Code Resolution
      else if (vars[regIndex].meta.type == TYPE_CODE) {
        src.meta.type = TYPE_CODE;
        src.meta.value = vars[regIndex].meta.value;
      }
      // Live Hardware Resolution
      else if (vars[regIndex].meta.type == TYPE_DEV) {
        int handle = vars[regIndex].meta.value;
        char d_type = allocated_devices[handle].type;
        uint8_t target_pin = allocated_devices[handle].pinA;

        if (d_type == 'I') {
          num = digitalRead(target_pin);
        } else if (d_type == 'A') {
          num = analogRead(target_pin);
        } else if (d_type == 'R') {
          num = mock_servos[target_pin]; 
        } else {
          num = allocated_devices[handle].shadow_value; 
        }
      } 
      // Standard Global Variable
      else {
        num = vars[regIndex].meta.value; 
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

  // Active Read Modifiers (I, U, A) - Read hardware, retain value in accumulator for assignment
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
    doop(); // Evaluate the condition first
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
  processChar('\n'); // Automatically execute any pending operations at End of File
}

#ifdef TEST
#include "abc_tests.h"
#endif

void setup() {
  DEBUG_SERIAL.begin(BAUD);
  delay(2000); 

#ifndef TEST
  #if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
  EEPROM.begin(1024);
  #endif
#endif

#ifdef TEST

  runAllTests();
  if (mock_eeprom[0] != '\0') {
    DEBUG_SERIAL.println("Executing Boot Script from Mock EEPROM...");
    evaluateABC(mock_eeprom);
  }
#else
  // REAL HARDWARE BOOT SEQUENCE
  char firstChar = EEPROM.read(0);
  if (firstChar != 0 && firstChar != 255) {
    DEBUG_SERIAL.println("Executing Boot Script from Hardware EEPROM...");
    int addr = 0;
    char c = EEPROM.read(addr);
    while (c != '\0' && c != 255 && addr < 1023) {
      code_ram[addr++] = c;
      c = EEPROM.read(addr);
    }
    code_ram[addr] = '\0';
    evaluateABC(code_ram);
  }
#endif

  // Initialize 't' as the default Terminal for the REPL
  vars['t'-'a'].meta.type = TYPE_DEV;
  vars['t'-'a'].meta.value = allocateDevice('T', 0);
  DEBUG_SERIAL.println("\n--- ABC Interactive REPL ---");
  in_repl = true;
}

void loop() {
  if (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();
    static char last_char = 0;

    // Ignore a Linefeed if it immediately follows a Carriage Return
    if (c == '\n' && last_char == '\r') {
      last_char = c;
      return;
    }

    // Handle Backspace (ASCII 8 or 127)
    if (c == '\b' || c == 127) {
      if (rx_head > rx_tail) {
        rx_head--; // Remove it from our ring buffer
        DEBUG_SERIAL.print("\b \b"); // Visually erase it from the terminal
      }
      return;
    }

    // Echo visually 
    if (c == '\r' || c == '\n') {
      DEBUG_SERIAL.print("\r\n");
    } else {
      DEBUG_SERIAL.print(c);
    }

    // ring buffer. also for string matching
    rx_buffer[rx_head % 64] = c;
    rx_head++;

    // Evaluate the line
    if (c == '\n' || c == '\r') {
      char repl_line[128]; // Isolate REPL commands from code_ram!
      int len = rx_head - rx_tail;
      
      for(int i = 0; i < len; i++) {
        repl_line[i] = rx_buffer[(rx_tail + i) % 64];
      }
      repl_line[len] = '\0';
      
      evaluateABC(repl_line); // Run it!
      
      rx_tail = rx_head; // Consume the buffer
      DEBUG_SERIAL.print("\r\n> "); // Print the next prompt
    }
    
    last_char = c;
  }
}