#include <Arduino.h>

/*
Goals 
- get close to a high level language, w/ understandable syntax, without including a compiler, 
- using the minimum resources possible to interpret the bytecodes. 
- The main pattern is: Destination, [Operation, Source ...], e.g. a=b+c-d. 
- The regular keywords in a language are single characters, making use of punctuation

0-9 A-F	nibble swap NUM, load hex digit into low nibble of NUM. 
	Conversion from/to decimal is too much? Maybe not?
:	Copy operation, e.g. "define"
a-z	set SRC or DST to register. a is register 0. b is register 4. z is 100.
	a:5 sets register 0 to 5. a:b copies register 4 to register 0.
	NUM is added to the register address first. 3a is the third byte after the start of a. 
	Register 125 is 7Da or 19z (19h=25d, z=100, +25=125) SRC, DST, etc start at 5z.
	After DST is loaded, SRC/DST is set and the next address is loaded to SRC.
@	index. Replace SRC or DST with the value at that address
	 and clear op. This sets the stage for another op and SRC.
	e.g. b@a sets the DST to the address of b plus the value of a.
	If the SRC is a port or port pin, read that value in. 
"	(Quote) Text. Each following char is copied to the DST until the ending quote.
	If the DST is a variable, the chars are actually copied into FLASH and the var is
	set to the starting address of the string in FLASH.
	If the operation was already " when a new starting " is seen, 
	put a " to the dest then enter text mode. "Push ""START""" prints Push "START"
#	??? Converts the value of source to decimal digits and copies it to DST.
	incrementing DST after each digit. Managed better by 'r' as radix?
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
(	parms. Prep for a function call by pushing parameters. 
)	call. Call the function pointed to by DST by incrementing PCP and loading DST to PC.
[	Start loop
]	End loop if true flag is not set.
.	return. Process OP/SRC, decrement PCP.
A	(Analog) set Port pin in DST to output PWM in SRC. e.g. P2A100
	set Port pin in SRC to read analog values in e.g. i:2P1A.
  Not available in hex mode. Use P1:2 or more instead? 
  Read analog by default on pins set to input which support ADC?
D	(Delay) DST microseconds between IO commands. Clears DST. e.g. 100DP0HLHL
  Not available in hex mode. Use 'W' for wait instead.
J	(Jump) move NUM lines ?

K	(Local) set SRC or DST to the LCD/Keys. 
	The actual value stored is 0x88
	NUM is used to select the position?
S	(Servo) set Port pin in DST to drive RC servo to postion in SRC. e.g. P1S90
T	(Terminal) set SRC or DST to the Serial port. 0x89
T ?Torque set torqen to SRC for servo in DST. e.g. P1T50 sets torque of servo on port 1 to 50%
M ?stepper Motor on <step_pin> <dir_pin> 
V ?motion profile for the stepper. <accelleration>, <velocity limit>
G ?goto. Move the stepper motor to the specified position.

P	(Port) set SRC or DST to IO pins. The value stored will be 0x80-0x87. e.g. 2P1 is port 2 pin 1
	NUM before P selects the port if more than 1 available. stored in the lower 3 bits of the value.
	NUM after P selects the pin. These are 1 to 8, not 0 to 7 so that 0 can indicate the entire port.
I	(In) set the Port or Port pin in SRC to an Input. E.g. a:2P7I@ reads port 2 pin 7 into a
O	(Out) set the Port or Port pin in DST to an Output (Can't H or L just do this?)
H	(High) set the Port pin(s) in DST to high. e.g. P1H sets port 0, pin 1 (the second pin) high.
L	(Low) set the Port pin(s) in DST to low. e.g. 2PL sets all pins on port 2 low.
	When the pin is an input, H and L set or clear TRUE based on the pins value.
U	(Up) set Port pin(s) in DST to inputs will internal pull-up
W	wait. Delay for DST u seconds. Not implemented.

Unused (for now)
$	
%	printf?
;	push?
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
long vars[26];      // Registers 'a' through 'z'
#define radix vars['r'-'a'] // Alias 'r' to radix
#define sp    vars['s'-'a'] // Alias 's' to stack pointer
#define pc    vars['p'-'a'] // Alias 'p' to program counter

long stack[256];            // The actual memory stack
int frame_pointer = 0;      // Points to start of arguments in the current call
int current_arg_count = 0;  // How many args in the current scope
int call_depth = 0;         // Are we inside a subroutine?

bool in_char_literal = false; // For parsing 'm'
long num = 0;                 // Numeric accumulator

// The Tagged Union structure for dynamically typed targets!
#define TYPE_NUM 0
#define TYPE_REG 1
#define TYPE_PIN 2
#define TYPE_DEV 3

union ABCState {
  long raw;              // Allows a full 32-bit push/pop to the stack
  struct {
    int16_t value;       // Payload: register index, pin number, device ID
    uint8_t type;        // TYPE_NUM, TYPE_REG, TYPE_PIN, TYPE_DEV
    uint8_t arg_count;   // Free byte! Useful for storing arg counts later
  } meta;
};

ABCState dst; // Current Destination
ABCState src; // Current Source

char op = 0;          // Current Operation
bool src_dst = false; // False = looking for DST, True = looking for SRC
bool true_flag = true; // For conditionals (?)
bool skip_line = false; 

// --- MOCK HARDWARE FOR TESTING ---
long pin_shadow[30];   // Stores digital (0/1) or PWM (>1) states
long mock_servos[30]; // Stores servo angles

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
  return &vars[regIndex];
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
        pin_shadow[dst.meta.value] = num; // Restore mock tracking for the test harness
        pinMode(dst.meta.value, OUTPUT);
        // Basic distinction: 0/1 is digital, > 1 is PWM
        if (num > 1) analogWrite(dst.meta.value, num);
        else digitalWrite(dst.meta.value, num);
      } else if (target) {
        *target = num;
      }
      break;
      
    case '+': if (target) *target += num; break;
    case '-': if (target) *target -= num; break;
    case '&': if (target) *target &= num; break;
    case '|': if (target) *target |= num; break;
    case '=': if (target) true_flag = (*target == num); break;
    case '<': if (target) true_flag = (*target < num); break;
    case '>': if (target) true_flag = (*target > num); break;
    case '{': if (target) true_flag = (*target <= num); break; // <=
    case '}': if (target) true_flag = (*target >= num); break; // >=
    
    case 'S': // Servo Output
      if (dst.meta.type == TYPE_PIN) {
        mock_servos[dst.meta.value] = num;
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

    // MOCK HARDWARE FUNCTION CALL ('t')
    if (src.meta.type == TYPE_REG && src.meta.value == ('t' - 'a')) {
      long device_type = stack[sp - current_arg_count]; // E.g., 'm'
      // ... Hardware Init ...
      ret_val = 99; // Return device handle
    }

    sp -= current_arg_count;
    current_arg_count = 0; 

    // Restore the VM state exactly as we left it!
    long state_flags = stack[--sp];
    src.raw = stack[--sp];
    dst.raw = stack[--sp];

    src_dst = (state_flags & 1) != 0;
    op = (state_flags >> 8) & 0xFF;

    num = ret_val; 
    return;
  }

  // Accumulate Numbers (Based on 'r' register radix)
  if (c >= '0' && c <= '9') {
    num *= radix;
    num += (c - '0');
    return;
  }
  if (radix == 16 && (c >= 'A' && c <= 'F')) {
    num *= radix;
    num += (c - 'A' + 10);
    return;
  }

  // Identify Variables / Registers (a-z)
  if (c >= 'a' && c <= 'z') {
    int regIndex = c - 'a';
    if (!src_dst) { // Looking for destination
      dst.meta.type = TYPE_REG;
      dst.meta.value = regIndex;
      src_dst = true; // Next variable is source
    } else {        
      src.meta.type = TYPE_REG;
      src.meta.value = regIndex;
      num = *getVarPtr(regIndex); // Load value into accumulator
    }
    return;
  }

  // Hardware Pin Modifier ('P')
  if (c == 'P') {
    if (!src_dst) { // Modifying destination
      dst.meta.type = TYPE_PIN;
      dst.meta.value = num;
      src_dst = true; 
      num = 0; // Reset accumulator ONLY for the destination
    } else {        // Modifying source        
      src.meta.type = TYPE_PIN;
      src.meta.value = num;
      // REAL HARDWARE READ!
      num = digitalRead(src.meta.value); 
      // Do NOT reset num here, we need it for the operation!
    }
    return;
  }

  // Immediate Hardware/State Operations (H, L, I, U)
  if (c == 'H' || c == 'L') {
    if (dst.meta.type == TYPE_PIN) {
      pin_shadow[dst.meta.value] = (c == 'H') ? 1 : 0; 
      pinMode(dst.meta.value, OUTPUT);
      digitalWrite(dst.meta.value, (c == 'H') ? HIGH : LOW);
    }
    num = 0; op = 0; src_dst = false; dst.raw = 0;
    return;
  }

  if (c == 'I') { // Input
    if (dst.meta.type == TYPE_PIN) pinMode(dst.meta.value, INPUT);
    num = 0; op = 0; src_dst = false; dst.raw = 0;
    return;
  }

  if (c == 'U') { // Input Pull-Up (Changed from P to avoid collision)
    if (dst.meta.type == TYPE_PIN) pinMode(dst.meta.value, INPUT_PULLUP);
    num = 0; op = 0; src_dst = false; dst.raw = 0;
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
    delayus(num);
    num = 0; op = 0; src_dst = false; dst.raw = 0;
    return;
  }

  // End of Line / Execution Trigger
  if (c == '\n' || c == '\r') {
    doop(); 
    num = 0; op = 0; src_dst = false; 
    dst.raw = 0; src.raw = 0; // Clean wipe!
    return;
  }

  // If it's none of the above, it's likely a standard operator
  if (c != ' ' && c != '\t') { 
    doop(); 
    op = c;
  }
}

void evaluateABC(const char* commands) {
  while (*commands) {
    processChar(*commands++);
  }
}

// --- TEST HARNESS FRAMEWORK ---
int testCount = 0;
int passCount = 0;


void resetTestState() {
  for(int i=0; i<26; i++) vars[i] = 0;
  radix = 10; 
  num = 0; 
  dst.raw = 0; 
  src.raw = 0; 
  op = 0; 
  src_dst = false; 
  true_flag = true; 
  skip_line = false;
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
  long actualValue = vars[checkReg - 'a'];
  if (actualValue == expectedValue) {
    DEBUG_SERIAL.printf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    DEBUG_SERIAL.printf("       Expected register '%c' to be %ld, but got %ld\r\n", checkReg, expectedValue, actualValue);
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
    Serial.printf("[FAIL] %s\r\n       Code: %s\r       Expected Pin %d to be %ld, but got %ld\r\n", 
                  testName, code, checkPin, expectedValue, actualValue);
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
  runHardwareTest("Digital Pin High", "13P H\n", 13, 1);

  // TEST 8: Digital Low Output
  runHardwareTest("Digital Pin Low", "9P L\n", 9, 0);

  // TEST 9: Analog/PWM Output
  runHardwareTest("Analog PWM Output", "5P:128\n", 5, 128);

  // TEST 10: Servo Output
  runHardwareTest("Servo Position", "2P S 90\n", 2, 90, true);

  // TEST 11: Radix Switching (Base 16)
  // Set radix to 16, then load hex "10" into 'a'. Expected decimal value: 16
  runTest("Radix Hex", "r:16\na:10\n", 'a', 16);

  // TEST 12: Radix Switching (Base 2)
  // Set radix to 2, then load binary "101" into 'a'. Expected decimal value: 5
  runTest("Radix Binary", "r:2\na:101\n", 'a', 5);

  // TEST 13: Single Quote ASCII parsing
  // Assign the ASCII value of 'm' (109) to 'a'
  runTest("Single Quote ASCII", "a:'m'\n", 'a', 109);

  // TEST 14: Comma Pushing & Stack Pointer Manipulation
  // Push 10, 20 to stack. The 's' register (SP) should increment by 2.
  runTest("Stack Pointer Update", "10, 20,\n", 's', 2);

  // TEST 15: Mock Hardware Device Handle
  // t is the hardware init function. 'm' is type motor. It returns handle 99 to 'a'.
  runTest("Hardware Function Call", "a:t('m', 3, 4)\n", 'a', 99);

  // TEST 16: Variable Parameter Shadowing
  // We manually spoof entering a subroutine with 2 arguments already on the stack.
  // 'a' should map to stack[0], and 'b' should map to stack[1]. 'c' maps to global.
  testCount++;
  for(int i=0; i<26; i++) vars[i] = 0;
  stack[0] = 77; // Spoof Arg 1
  stack[1] = 88; // Spoof Arg 2
  sp = 2;
  frame_pointer = 0;
  current_arg_count = 2;
  call_depth = 1; // SPOOF WE ARE INSIDE A FUNCTION
  
  // Try to copy 'a' to 'c'. If shadowing works, 'c' becomes 77, not 0!
  evaluateABC("c:a\n"); 
  if (vars['c'-'a'] == 77) {
      Serial.printf("[PASS] Variable Parameter Shadowing\r\n");
      passCount++;
  } else {
      Serial.printf("[FAIL] Variable Parameter Shadowing\r\n");
  }
  call_depth = 0; // reset

  // --- HARDWARE IN THE LOOP (HIL) TESTS ---
  // Note: Requires Wokwi diagram connecting GPIO 2 to GPIO 3. Let's check:
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  delay(10); 
  if (digitalRead(3) == HIGH) {
    DEBUG_SERIAL.println("[PASS] Native C++ Wiring Check (GP2 -> GP3 connected)");
  } else {
    DEBUG_SERIAL.println("[FAIL] Simulator not setup! Please check diagram.json.");
  }
  digitalWrite(2, LOW); // Reset

  // 1. Set Pin 3 to Input
  evaluateABC("3P I\n");

  // 2. Set Pin 2 to High, wait 1000us (1ms), then read Pin 3 into 'a'
  runTest("HIL: GPIO High Read", "2P H\n1000 W\na:3P\n", 'a', 1);

  // 3. Set Pin 2 to Low, wait 1000us (1ms), then read Pin 3 into 'b'
  runTest("HIL: GPIO Low Read", "2P L\n1000 W\nb:3P\n", 'b', 0);
  
  // 4. Test Assignment digital write (0/1) instead of H/L
  runTest("HIL: GPIO Assign High", "2P:1\n1000 W\nc:3P\n", 'c', 1);
  
  DEBUG_SERIAL.printf("\r\n--- Test Run Complete: %d/%d Passed ---\r\n", passCount, testCount);
}

//TODO: Add basic hardware simulation via a WOKWI filter driven by an "analog" PWM output and reading an analog input. 
//      https://wokwi.com/projects/409325290405496833

void loop() {
  //halt and look for input
  String input = DEBUG_SERIAL.readStringUntil('\n'); // Wait for user input 
  if (input.length() > 0) {
    DEBUG_SERIAL.printf("Evaluating ABC code:\n%s\n", input.c_str());
    evaluateABC(input.c_str());
    DEBUG_SERIAL.println("Done evaluating.\n");
  }
}