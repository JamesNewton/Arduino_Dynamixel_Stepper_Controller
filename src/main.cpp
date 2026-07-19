#include <Arduino.h>
#include <Wire.h> // I2C support.
#include <Servo.h>
#define TEST // Comment this line out to compile for REAL hardware!

#ifndef TEST
#include <EEPROM.h>
#endif

#define BAUD 115200
//#define BAUD 57600 //Default Dynamixel baudrate
//The same baud rate should be used for both the Arduino and the Dynamixel. e.g. if you want to talk to the
//Arduino at 115200, then you need to re-program the Dynamixel to be at 115200. Of course, if you don't use
//the Dynamixels, you can set this to whatever you like. 

//#define DYNAMIXEL_SUPPORT
//Note: If enabled, the Dynamixel Arduino shield is required and the standard Arduino UNO serial port is NOT functional! 
//You must install the USB to Serial interface on the Dynamixel Shield and use that for communication. RTFM

//#define STEPPER_SUPPORT

//#define ENCODER_SUPPORT
//#define USE_ANALOG_ENCODER // Comment this out to fall back to standard digital Encoder.h

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

#define DYNAMIXEL_SERVO_ID 1
#define DYNAMIXEL_CTRL_TABLE_LENGTH 256

#define DYNAMIXEL_SERVO_MODE OP_EXTENDED_POSITION
//https://emanual.robotis.com/docs/en/popup/arduino_api/setOperatingMode/

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
int dynamixel_servo_id;
using namespace ControlTableItem;
#else
#define DXL_DIR_PIN -1
#define DEBUG_SERIAL Serial
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

#define STACK_SIZE 256
long stack[STACK_SIZE];            // The actual memory stack
int frame_pointer = 0;      // Points to start of arguments in the current call
int current_arg_count = 0;  // How many args in the current scope
int call_depth = 0;         // Are we inside a subroutine?
int scope_frame_pointer = 0;
int scope_arg_count = 0;
#define CODE_SIZE 1024
char code_ram[CODE_SIZE]; // Volatile dictionary for user functions
int code_ptr = 0;
bool quote_pending = false; // For detecting "" escapes
bool in_string_literal = false;
bool in_char_literal = false; // For parsing 'm'
long num = 0;                 // Numeric accumulator
int format_width = 0; 
int format_prec = 0;

ABCState dst; // Current Destination
ABCState src; // Current Source

char op = 0;          // Current Operation
bool src_dst = false; // False = looking for DST, True = looking for SRC
bool true_flag = true; // For conditionals (?)
bool skip_line = false; 
long active_sub_addr = -1; // Tracks the @ operator address

// --- EXECUTION & LOOP CONTROL ---
const char* pc_ptr = nullptr; // Tracks our current position in the bytecode string
#define MAX_LOOP_DEPTH 8
const char* loop_stack[MAX_LOOP_DEPTH];    // Supports nested loops up to 8 levels deep
int loop_depth = 0;           // Current loop nesting level

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
#ifdef DYNAMIXEL_SUPPORT
long mock_dxl_ram[MAX_DEVICES][DYNAMIXEL_CTRL_TABLE_LENGTH]; 
// Simulates control table
#endif
#endif
int eeprom_ptr = 0;
Servo physical_servos[NUM_DIGITAL_PINS]; 

#ifdef STEPPER_SUPPORT
#include <AccelStepper.h>
// Array of pointers mapped to the Device Handle IDs
AccelStepper* physical_steppers[MAX_DEVICES] = {nullptr}; 
#endif

#ifdef ENCODER_SUPPORT
    // --- HYBRID ANALOG ENCODER ---
// https://github.com/JamesNewton/HybridDiskEncoder/tree/master#hybrid-disk-encoder
  class AnalogEncoder {
    private:
      uint8_t pin_sin, pin_cos;
      int32_t count;
      int8_t old_state;
      int32_t sincenter, coscenter;
// Quadrature Encoder Matrix
// Indexed by the bits AoBoAnBn 
//Where A and B are the encoder outputs and o and n are old and new. 
//e.g. Ao is the OLD value of A. See the read_encoder function below.
//  0 = nop: no change
//  1 = inc: increment (positive rotation)
// -1 = dec: decrement (negative rotation)
//  2 = err: error (both changed, so we missed a state)
      int QEM [16] = { 
//old new:0  1  2  3 //
/*0=00*/  0,-1, 1, 2,// 0->0? nop, 1->0? dec, 2->0? inc, 3->0? err
/*1=01*/  1, 0, 2,-1,// 0->1? inc, 1->1? nop, 2->1? err, 3->1? dec
/*2=02*/ -1, 2, 0, 1,// 0->2? dec, 1->2? err, 2->2? nop, 3->2? inc
/*3=03*/  2, 1,-1, 0 // 0->3? err, 1->3? inc, 2->3? dec, 3->3? nop
      }; 
//notice how the err 2's are on the diagonal?
//and the nop 0's are on the other diagonal?
//and the inc and dec 1's form a sort of circle?
        
    public:
      AnalogEncoder(uint8_t sinPin, uint8_t cosPin, int32_t sCenter = 512, int32_t cCenter = 512) {
        pin_sin = sinPin;
        pin_cos = cosPin;
        sincenter = sCenter;
        coscenter = cCenter;
        count = 0;
        old_state = 0;
      }
      
      // Analog polling requires active updates!
      void update() {
        int sin_sign = analogRead(pin_sin) - sincenter;
        int cos_sign = analogRead(pin_cos) - coscenter;
        int8_t new_state = (sin_sign > 0) + 2 * (cos_sign > 0);
  // 0 = 00 = a0 + b0
  // 1 = 01 = a1 + b0
  // 2 = 10 = a0 + b2
  // 3 = 11 = a1 + b2
  //index the Quadrature Encoder Matrix by old and new state
  //make the old state the 4th and 5th bits
        int8_t action = QEM[new_state + 4 * old_state];
        old_state = new_state;
        if (action != 2) count += action; 
      }
      
      long read() { 
        int sin_sign = analogRead(pin_sin) - sincenter;
        int cos_sign = analogRead(pin_cos) - coscenter;
        long a = (atan2(sin_sign, cos_sign) / (2 * M_PI)) * 1024 + 512;
        // Combine the coarse count with the fine sub-step phase
        return (count * 1024) + a; 
      }
      void write(long p) { count = p; }
    // end of public methods
  };
  typedef AnalogEncoder ABC_Encoder;
  ABC_Encoder* physical_encoders[MAX_DEVICES] = {nullptr};
#endif

inline void vm_yield() {
  //sleep_us(1); 
#ifdef STEPPER_SUPPORT
  for (int i = 1; i < next_device_index; i++) {
    if (allocated_devices[i].type == 'S' && physical_steppers[i] != nullptr) {
      //DEBUG_SERIAL.print(".");
      physical_steppers[i]->run();
    }
  }
#endif
#if defined(ENCODER_SUPPORT) && defined(USE_ANALOG_ENCODER)
  for (int i = 1; i < next_device_index; i++) {
    if (allocated_devices[i].type == 'Q' && physical_encoders[i] != nullptr) {
      physical_encoders[i]->update(); // Poll the ADCs!
    }
  }
#endif
  // Future: Could add Dynamixel
}

void delayus(unsigned long us) {
  unsigned long start = micros();
  while (micros() - start < us) {
    vm_yield();
  }
}

#ifdef DYNAMIXEL_SUPPORT
void rebootDynamixelServo(int id, int mode) { //setup servo id number into mode.
  dxl.torqueOff(id);
  if (!mode) mode = DYNAMIXEL_SERVO_MODE;
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
  if (call_depth > 0 && regIndex < scope_arg_count) {
    return &stack[scope_frame_pointer + regIndex];
  }
  return (long*)&vars[regIndex].meta.value;
}

bool matchStream(const char* target_str) {
  int peek_tail = rx_tail; // Create a temporary cursor to peek ahead
  int i = 0;

  while (target_str[i] != '\0') {
    // 1. Block character-by-character until we have something to evaluate
    while (peek_tail == rx_head) {
      vm_yield();
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

void vm_error(const char* msg) {
  if (in_repl) {
    DEBUG_SERIAL.print("\r\n[ERR] ");
    DEBUG_SERIAL.println(msg);
    DEBUG_SERIAL.print("> "); // Reprint the prompt
  }
  // (Optional) We could also set a reserved register like 'e' to an error code here!
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
  if (next_device_index >= MAX_DEVICES) { vm_error("Max Devices"); return 0;}
  allocated_devices[next_device_index].type = dev_type;
  switch (dev_type) {
    case 'A': //Analog Input. Arguments: (Type, Pin) or use A.
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      pinMode(allocated_devices[next_device_index].pinA, INPUT);
      break;
    case 'F': //EEPROM. Arguments: (Type) or use F.
      eeprom_ptr = 0; 
#ifdef TEST
      mock_eeprom[0] = '\0';
#endif
      break;
    case 'i': //I2C Bus. Arguments: (Type, SclPin, SdaPin)
      allocated_devices[next_device_index].pinA = stack[arg_start + 1]; // SCL
      allocated_devices[next_device_index].pinB = stack[arg_start + 2]; // SDA
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 3]; // I2C Target Address
      // On RP2040, we map the I2C pins dynamically
      #if defined(ARDUINO_ARCH_RP2040)
      Wire.setSCL(allocated_devices[next_device_index].pinA);
      Wire.setSDA(allocated_devices[next_device_index].pinB);
      #endif
      Wire.begin();
      break;
    case 'I': //Digital Input. Arguments: (Type, Pin, Pull) or use I, or U.
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      if (stack[arg_start + 2] == 1) pinMode(allocated_devices[next_device_index].pinA, INPUT_PULLUP);
      else pinMode(allocated_devices[next_device_index].pinA, INPUT);
      break;
    case 'O': //Digital Output. Arguments: (Type, Pin, Value) or use H, L.
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
      pinMode(allocated_devices[next_device_index].pinA, OUTPUT);
      digitalWrite(allocated_devices[next_device_index].pinA, allocated_devices[next_device_index].shadow_value);
      break;
    case 'P': //PWM Output. Arguments: (Type, Pin, Value) or use P.
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
      pinMode(allocated_devices[next_device_index].pinA, OUTPUT);
      analogWrite(allocated_devices[next_device_index].pinA, allocated_devices[next_device_index].shadow_value);
      break;
    case 'Q': // Analog Quadrature Encoder
#ifdef ENCODER_SUPPORT
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].pinB = stack[arg_start + 2];
      allocated_devices[next_device_index].shadow_value = 0;
      {
        long s_center = (current_arg_count >= 4) ? stack[arg_start + 3] : 512;
        long c_center = (current_arg_count >= 5) ? stack[arg_start + 4] : 512;

        if (physical_encoders[next_device_index] != nullptr) {
          delete physical_encoders[next_device_index];
        }
        physical_encoders[next_device_index] = new ABC_Encoder(
          allocated_devices[next_device_index].pinA, 
          allocated_devices[next_device_index].pinB,
          s_center,
          c_center
        );
      }
#else
      vm_error("Encoder support disabled in firmware");
      return 0; // Fail the allocation
#endif
      break;
    case 'R': // Servo. Arguments: (Type, Pin, Value) or use R.
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].shadow_value = stack[arg_start + 2];
      physical_servos[allocated_devices[next_device_index].pinA].attach(allocated_devices[next_device_index].pinA);
      physical_servos[allocated_devices[next_device_index].pinA].write(allocated_devices[next_device_index].shadow_value);
      break;
    case 'S': // Stepper Motor. Arguments: (Type, StepPin, DirPin, MaxVel, Accel).
#ifdef STEPPER_SUPPORT
      //TODO: check current_arg_count to ensure we have enough parameters.
      allocated_devices[next_device_index].pinA = stack[arg_start + 1];
      allocated_devices[next_device_index].pinB = stack[arg_start + 2];
      allocated_devices[next_device_index].shadow_value = 0; // Tracks target
      // AccelStepper::DRIVER (1) indicates a dedicated Step/Dir driver
      if (physical_steppers[next_device_index] != nullptr) {
        delete physical_steppers[next_device_index];
      }
      physical_steppers[next_device_index] = new AccelStepper(
        AccelStepper::DRIVER, 
        allocated_devices[next_device_index].pinA, 
        allocated_devices[next_device_index].pinB
        );
      physical_steppers[next_device_index]->setMaxSpeed(stack[arg_start + 3]);
      physical_steppers[next_device_index]->setAcceleration(stack[arg_start + 4]);
#else
      vm_error("Stepper support disabled in firmware");
      return 0;
#endif
      break;
    case 'D': //Dynamixel Servo. Arguments: (Type, ServoID, Baudrate)
#ifdef DYNAMIXEL_SUPPORT
      allocated_devices[next_device_index].pinA = stack[arg_start + 1]; // Servo ID
      allocated_devices[next_device_index].shadow_value = 0; 
      dxl.begin(stack[arg_start + 2]); // Baudrate
      dxl.setPortProtocolVersion(2.0);
      dxl.ping(allocated_devices[next_device_index].pinA);
      
      // Standard Boot: Torque off, set Position mode, Torque on
      dxl.torqueOff(allocated_devices[next_device_index].pinA);
      dxl.setOperatingMode(allocated_devices[next_device_index].pinA, OP_POSITION);
      dxl.torqueOn(allocated_devices[next_device_index].pinA);
#else
      vm_error("Dynamixel support disabled in firmware");
      return 0;
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
  //DEBUG_SERIAL.printf("[RETURN] ret_val=%ld. returning to caller depth=%d\r\n", ret_val, call_depth);
  // Restore lexical scope to the caller
  if (call_depth > 0) {
    scope_frame_pointer = frame_pointer;
    scope_arg_count = current_arg_count;
  } else {
    scope_frame_pointer = 0;
    scope_arg_count = 0;
  }
}

long readDeviceState(int handle) {
  char d_type = allocated_devices[handle].type;
  uint8_t target_pin = allocated_devices[handle].pinA;
  long result = 0;

  if (d_type == 'I') {
    result = digitalRead(target_pin);
  } else if (d_type == 'A') {
    result = analogRead(target_pin);
  } else if (d_type == 'R') {
    result = physical_servos[target_pin].read();
  } else if (d_type == 'Q') {
#ifdef ENCODER_SUPPORT
    result = physical_encoders[handle] ? physical_encoders[handle]->read() : 0;
#endif
  } else if (d_type == 'S') {
#ifdef STEPPER_SUPPORT
    result = physical_steppers[handle] ? physical_steppers[handle]->currentPosition() : 0;
#endif
  } else if (d_type == 'D') {
#ifdef DYNAMIXEL_SUPPORT
    long read_addr = (active_sub_addr != -1) ? active_sub_addr : 132;
#ifdef TEST
    result = mock_dxl_ram[handle][read_addr % 256];
#else
    result = dxl.readControlTableItem(read_addr, target_pin);
#endif
#endif
    active_sub_addr = -1; // Consume the address!
  } else if (d_type == 'i') {
    long read_addr = (active_sub_addr != -1) ? active_sub_addr : 0; 
    int bytes_to_read = (num > 0) ? num : 1; 
    int i2c_addr = allocated_devices[handle].shadow_value;
    
    Wire.beginTransmission(i2c_addr);
    Wire.write(read_addr);
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_addr, bytes_to_read);
    
    if (bytes_to_read == 1) {
      result = Wire.available() ? Wire.read() : 0;
    } else {
      int start_ptr = code_ptr;
      if (code_ptr + bytes_to_read >= CODE_SIZE) {
        vm_error("Code RAM Overflow");
        bytes_to_read = CODE_SIZE - code_ptr - 1; // Prevent overflow
      }
      for(int i = 0; i < bytes_to_read; i++) {
        code_ram[code_ptr++] = Wire.available() ? Wire.read() : 0;
      }
      src.meta.type = TYPE_CODE;
      src.meta.value = start_ptr;
      result = start_ptr; 
    }
    active_sub_addr = -1; // Consume the address!
  } else {
    result = allocated_devices[handle].shadow_value; 
  }
  
  return result;
}

void writeDeviceState(int handle, long val) {
  char d_type = allocated_devices[handle].type;
  uint8_t target_pin = allocated_devices[handle].pinA;

  // I2C handles its own shadow value (bus address), everything else updates the tracker
  if (d_type != 'i') {
    allocated_devices[handle].shadow_value = val; 
  }

  if (d_type == 'O') {
    digitalWrite(target_pin, val);
  } else if (d_type == 'P') {
    analogWrite(target_pin, val);
  } else if (d_type == 'R') {
    physical_servos[target_pin].write(val);
  } else if (d_type == 'Q') {
#ifdef ENCODER_SUPPORT
    if (physical_encoders[handle]) physical_encoders[handle]->write(val);
#endif
  } else if (d_type == 'S') {
#ifdef STEPPER_SUPPORT
    if (physical_steppers[handle]) physical_steppers[handle]->moveTo(val);
#endif
  } else if (d_type == 'D') {
#ifdef DYNAMIXEL_SUPPORT
    long write_addr = (active_sub_addr != -1) ? active_sub_addr : 116; 
    allocated_devices[handle].shadow_value = val;
#ifdef TEST
    mock_dxl_ram[handle][write_addr % 256] = val;
    if (write_addr == 116) mock_dxl_ram[handle][132] = val; 
#else
    dxl.writeControlTableItem(write_addr, target_pin, val);
#endif
#endif
    active_sub_addr = -1; 
  } else if (d_type == 'i') {
    long write_addr = (active_sub_addr != -1) ? active_sub_addr : 0;
    int i2c_addr = allocated_devices[handle].shadow_value;
    Wire.beginTransmission(i2c_addr);
    Wire.write(write_addr);
    Wire.write(val);
    Wire.endTransmission();
    active_sub_addr = -1; 
  }
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
          if (!physical_servos[dst.meta.value].attached()) {
            physical_servos[dst.meta.value].attach(dst.meta.value);
          }
          physical_servos[dst.meta.value].write(num);
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
          writeDeviceState(handle, num); 
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
          } else { //src is not DEV or CODE. Must be a device?
            writeDeviceState(handle, num);
          } 
        } else { // No device attached.
          if (active_sub_addr != -1) {
            // RAM DEREFERENCE: active_sub_addr holds the base pointer, num holds the offset.
            num = code_ram[active_sub_addr + num];
            //DEBUG_SERIAL.printf("[RAM DEREF] BasePtr: %ld, Offset: %ld, Got: %d\r\n", active_sub_addr, num, code_ram[active_sub_addr + num]);
            active_sub_addr = -1;
          }
          if (src.meta.type == TYPE_DEV || src.meta.type == TYPE_CODE) {
            vars[dst.meta.value].meta.type = src.meta.type;
            vars[dst.meta.value].meta.value = src.meta.value;
          } else {
            *target = num; 
          }
        }
      }
      break;

    case '%': // FORMATTING OPERATOR
      {
        int handle = -1;
        if (dst.meta.type == TYPE_DEV) handle = dst.meta.value;
        else if (dst.meta.type == TYPE_REG && vars[dst.meta.value].meta.type == TYPE_DEV) {
          handle = vars[dst.meta.value].meta.value;
        }
        
        if (handle != -1) {
          if (src.meta.type == TYPE_CODE) {
            devWrite(handle, '"');
            const char* str = &code_ram[src.meta.value];
            while (*str) devWrite(handle, *str++);
            devWrite(handle, '"');
          } else {
            // FIXED-POINT VIRTUAL DECIMAL FORMATTING
            bool is_neg = (num < 0);
            long val = is_neg ? -num : num;
            
            char rawStr[33];
            ltoa(val, rawStr, radix);
            int rawLen = strlen(rawStr);
            
            char numBuf[64];
            int nIdx = 0;
            if (is_neg) numBuf[nIdx++] = '-';
            
            // Integer Part
            if (rawLen <= format_prec) {
              numBuf[nIdx++] = '0'; // Leading zero before decimal
            } else {
              for (int i = 0; i < rawLen - format_prec; i++) {
                numBuf[nIdx++] = rawStr[i];
              }
            }
            
            // Fractional Part
            if (format_prec > 0) {
              numBuf[nIdx++] = '.';
              // Fractional leading zeros (e.g., turning 5 into .05)
              for (int i = 0; i < format_prec - rawLen; i++) {
                numBuf[nIdx++] = '0';
              }
              // Fractional trailing numbers
              int start_idx = rawLen > format_prec ? rawLen - format_prec : 0;
              for (int i = start_idx; i < rawLen; i++) {
                numBuf[nIdx++] = rawStr[i];
              }
            }
            numBuf[nIdx] = '\0';
            
            // Apply Padding Width
            int numLen = strlen(numBuf);
            int pad = format_width - numLen;
            char outStr[64];
            int outIdx = 0;
            while (pad > 0) {
              outStr[outIdx++] = ' ';
              pad--;
            }
            for (int i = 0; i < numLen; i++) {
              outStr[outIdx++] = numBuf[i];
            }
            outStr[outIdx] = '\0';
            
            // Stream it to the device
            for (int i = 0; outStr[i] != '\0'; i++) devWrite(handle, outStr[i]);           
            format_width = 0; 
            format_prec = 0; 
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
        pin_type[dst.meta.value] = 'R';
        if (!physical_servos[dst.meta.value].attached()) {
          physical_servos[dst.meta.value].attach(dst.meta.value);
        }
        physical_servos[dst.meta.value].write(num);
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

  // Device Address Indexing Modifier (@)
  if (c == '@') {
    active_sub_addr = num;
    num = 0; // Clear accumulator for the upcoming device handle
    src.raw = 0;
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

  // Double Quote String Parsing & Escaping
  if (in_string_literal && c == '"') {
    if (quote_pending) {
      if (code_ptr >= CODE_SIZE - 1) { vm_error("RAM OOM"); in_string_literal = false; return; }
      code_ram[code_ptr++] = '"'; // Escaped quote
      quote_pending = false;
    } else {
      quote_pending = true; // Wait to see if the next char is a quote or the end
    }
    return;
  } else if (in_string_literal) {
    if (quote_pending) {
      if (code_ptr >= CODE_SIZE - 1) { vm_error("RAM OOM"); in_string_literal = false; return; }
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
      if (code_ptr >= CODE_SIZE - 1) { vm_error("RAM OOM"); in_string_literal = false; return; }
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
    if (loop_depth >= MAX_LOOP_DEPTH) {
      vm_error("Loop Overflow");
      return;
    }
    loop_stack[loop_depth++] = pc_ptr; // Save the address of the '[' character
    return;
  }

  if (c == ']') {
    doop(); // Execute any pending operations before evaluating the loop exit
    if (true_flag) {
      if (loop_depth > 0) {
        // Jump back! (The evaluateABC loop will increment this by 1, putting us right after '[')
        pc_ptr = loop_stack[loop_depth - 1]; 
      } else {
        vm_error("Loop Underflow");
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
    if (sp >= STACK_SIZE - 5) {
      vm_error("Stack Overflow");
      return;
    }
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
    //DEBUG_SERIAL.printf("[CALL START] Pushed frame=%d, arg_count=%d. New sp=%d\r\n", frame_pointer, current_arg_count, sp);
    frame_pointer = sp; // New frame starts here
    src_dst = true;     // Ensure arguments are evaluated as sources
    current_arg_count = 0;
    num = 0;
    return;
  }

  // Stack Operators: Push Arg
  if (c == ',') {
    if (sp >= STACK_SIZE - 1) {
      vm_error("Stack Overflow");
      return;
    }
    stack[sp++] = num;
    current_arg_count++;
    num = 0; op = 0; src_dst = false;
    src_dst = true; // Ensure the NEXT argument is also a source
    return;
  }

  // Stack Operators: Execute Call
  if (c == ')') {
    if (sp <= 0) {
      vm_error("Stack Underflow");
      return;
    }
    stack[sp++] = num;
    current_arg_count++;
    // Retrieve the function pointer we saved on the stack
    ABCState func_src;
    func_src.raw = stack[frame_pointer - 4];
    //DEBUG_SERIAL.printf("[CALL EXEC] Passed num=%ld. Jmp to ptr=%d. scope_arg_count=%d\r\n", num, func_src.meta.value, current_arg_count);
    // COMPLEX DEVICE SYSTEM CALL MANAGER ('D')
    if (func_src.meta.type == TYPE_FUNC && func_src.meta.value == 'D') {
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
    else if (func_src.meta.type == TYPE_CODE) {
      stack[sp++] = (long)pc_ptr; // Push Return Address to the stack
      
      // OFFSET BY -1 because the evaluateABC loop will automatically do pc_ptr++ right after this
      pc_ptr = &code_ram[func_src.meta.value] - 1; 
      
      // Lock in the lexical scope for the new function!
      scope_frame_pointer = frame_pointer;
      scope_arg_count = current_arg_count;
      
      // Clean the active VM state so the function starts fresh
      op = 0; src_dst = false; dst.raw = 0; src.raw = 0; num = 0;
      current_arg_count = 0;
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
  // Formatting Operator Modifier (%)
  if (c == '%') {
    if (op == ':') {
      op = 0; // Cancel the standard assignment, upgrade it to a format operation
    } else {
      doop(); // Execute any other pending math
    }
    format_width = num; 
    num = 0;            
    op = '%';           
    src_dst = true;     
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
      if (op == '%') {
        format_prec = num; // Catch the precision, if any, placed after the %
      }
      src.meta.type = TYPE_REG;
      src.meta.value = regIndex;
      
      // Parameter Shadowing:
      // If inside a function and index is a parameter, strictly use parameter value
      if (call_depth > 0 && regIndex < scope_arg_count) {
        num = stack[scope_frame_pointer + regIndex];
      }
      // Special Register 'q' - Queue Length
      else if (regIndex == 'q' - 'a') {
        num = rx_head - rx_tail;
      }
      // Global Code Resolution
      else if (vars[regIndex].meta.type == TYPE_CODE) {
        src.meta.type = TYPE_CODE;
        src.meta.value = vars[regIndex].meta.value;
        num = vars[regIndex].meta.value; // Allow pointers to be offset!
      }
      // Live Hardware Resolution
      else if (vars[regIndex].meta.type == TYPE_DEV) {
        int handle = vars[regIndex].meta.value;
        num = readDeviceState(handle);
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
    if (num > NUM_DIGITAL_PINS) {vm_error("Pin Overflow");return;}
    if (!src_dst) { 
      dst.meta.type = TYPE_PIN;
      dst.meta.value = num;
      if (c == 'R') pin_type[num] = 'R'; // Pre-mark it as a servo
      src_dst = true; 
      num = 0; // Reset accumulator ONLY for the destination
    } else {        
      src.meta.type = TYPE_PIN;
      src.meta.value = num;
      num = (c == 'R') ? physical_servos[num].read() : pin_shadow[num]; 
    }
    return;
  }

  // Immediate Hardware Output Commands (H, L)
  if (c == 'H' || c == 'L') {
    if (num > NUM_DIGITAL_PINS) {vm_error("Pin Overflow");num = 0;return;}
    pin_shadow[num] = (c == 'H') ? 1 : 0; 
    pin_type[num] = 'O';
    pinMode(num, OUTPUT);
    digitalWrite(num, (c == 'H') ? HIGH : LOW);
    num = 0; op = 0; src_dst = false; dst.raw = 0;
    return;
  }

  // Active Read Modifiers (I, U, A) - Read hardware, retain value in accumulator for assignment
  if (c == 'I') { // Digital Input
    if (num > NUM_DIGITAL_PINS) {vm_error("Pin Overflow");num = 0;return;}
    pin_type[num] = 'I';
    pinMode(num, INPUT);
    num = digitalRead(num); 
    return;
  }

  if (c == 'U') { // Input Pull-Up
    if (num > NUM_DIGITAL_PINS) {vm_error("Pin Overflow");num = 0;return;}
    pin_type[num] = 'u';
    pinMode(num, INPUT_PULLUP);
    num = digitalRead(num); 
    return;
  }

  if (c == 'A') { // Analog Input 
    if (num > NUM_DIGITAL_PINS) {vm_error("Pin Overflow");num = 0;return;}
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
    vm_yield();
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
    while (c != '\0' && c != 255 && addr < CODE_SIZE - 1) {
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