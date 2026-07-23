// abc_tests.h
#ifndef ABC_TESTS_H
#define ABC_TESTS_H

// This file expects to be included at the bottom of main.cpp, 
// so it has native access to the VM globals and evaluateABC().

int testCount = 0;
int passCount = 0;

void resetTestState(bool wipe_eeprom = true) {
for (int i = 0; i < MAX_DEVICES; i++) {
    allocated_devices[i].type = 0;
    
#ifdef STEPPER_SUPPORT
    if (physical_steppers[i] != nullptr) {
      delete physical_steppers[i];
      physical_steppers[i] = nullptr;
    }
#endif

#ifdef ENCODER_SUPPORT
    if (physical_encoders[i] != nullptr) {
      delete physical_encoders[i];
      physical_encoders[i] = nullptr;
    }
#endif

#ifdef DYNAMIXEL_SUPPORT
    for (int j = 0; j < DYNAMIXEL_CTRL_TABLE_LENGTH; j++) {
      mock_dxl_ram[i][j] = 0;
    }
#endif
  }
  next_device_index = 1;
  for(int i=0; i<26; i++) {
    vars[i].raw = 0;
    vars[i].meta.type = TYPE_REG; // Set type metadata
  }
  for(int i=0; i<NUM_DIGITAL_PINS; i++) {
    pin_shadow[i] = 0;
    pin_type[i] = 0;
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
  code_ptr = 0; 
  frame_pointer = 0;
  current_arg_count = 0;
  call_depth = 0;
  scope_frame_pointer = 0;
  scope_arg_count = 0;
  quote_pending = false; 
  in_string_literal = false;
  in_hash_literal = false;
  format_width = 0;
  format_prec = 0;
  mock_out_ptr = 0;
  mock_out_buffer[0] = '\0';
  mock_out_ptr = 0;
  mock_out_buffer[0] = '\0';
  
  if (wipe_eeprom) {
    eeprom_ptr = 0;
    mock_eeprom[0] = '\0';
  }
  // Set 't' as the default Terminal Device for all executions
  vars['t'-'a'].meta.type = TYPE_DEV;
  vars['t'-'a'].meta.value = allocateDevice('T', 0);
}

// --- MATCH STREAM TESTS ---

void setupMatchTest(const char* input, int start_tail = 0) {
  rx_tail = start_tail;
  rx_head = start_tail;
  while(*input) {
    rx_buffer[rx_head % 64] = *input++;
    rx_head++;
  }
}

void runMatchTest(const char* testName, const char* target, bool expectedResult, int expectedTail) {
  testCount++;
  bool result = matchStream(target);
  
  if (result == expectedResult && rx_tail == expectedTail) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    debugPrintf("       Expected Result: %d, Got: %d\r\n", expectedResult, result);
    debugPrintf("       Expected Tail: %d, Got: %d\r\n", expectedTail, rx_tail);
  }
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
  evaluateABC(code);

  long actualValue = vars[checkReg - 'a'].meta.value;
  if (actualValue == expectedValue) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    debugPrintf("       '%c' should be %ld, got %ld\r\n", checkReg, expectedValue, actualValue);
  }
}

void runAnalogTest(const char* testName, const char* code, char checkReg, long expectedValue, long tolerance) {
  testCount++;
  resetTestState();
  evaluateABC(code);
  long actualValue = vars[checkReg - 'a'].meta.value;
  if (abs(actualValue - expectedValue) <= tolerance) {
    debugPrintf("[PASS] %s (%ld, %ld±%ld)\r\n", testName, actualValue, expectedValue, tolerance);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    debugPrintf("       '%c' should be %ld (±%ld), got %ld\r\n", checkReg, expectedValue, tolerance, actualValue);
  }
}

void runHardwareTest(const char* testName, const char* code, int checkPin, long expectedValue, bool isServo = false) {
  testCount++;
  resetTestState();
  evaluateABC(code);

  long actualValue = isServo ? physical_servos[checkPin].read() : pin_shadow[checkPin];
  
  if (actualValue == expectedValue) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n       Code: %s\r       Expected Pin %d to be %ld, but got %ld\r\n", testName, code, checkPin, expectedValue, actualValue);
  }
}

void runStringTest(const char* testName, const char* code, const char* expectedString) {
  testCount++;
  resetTestState();
  evaluateABC(code);
  
  if (strcmp(mock_out_buffer, expectedString) == 0) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    debugPrintf("       Expected output: '%s', but got '%s'\r\n", expectedString, mock_out_buffer);
  }
}

void runFlashTest(const char* testName, const char* code, const char* expectedString) {
  testCount++;
  resetTestState(true); // Wipe everything including EEPROM
  evaluateABC(code);
  
  if (strcmp(mock_eeprom, expectedString) == 0) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    debugPrintf("       Expected EEPROM: '%s', but got '%s'\r\n", expectedString, mock_eeprom);
  }
}

void runBootTest(const char* testName, const char* bootScript, const char* userCode, char checkReg, long expectedValue) {
  testCount++;
  
  // 1. Hard reset and load the boot script into EEPROM
  resetTestState(true); 
  strcpy(mock_eeprom, bootScript);
  
  // 2. Simulate a power cycle! (Reset VM state, but DO NOT wipe EEPROM)
  resetTestState(false); 
  
  // 3. The Bootloader Hook (Run EEPROM if it exists)
  if (mock_eeprom[0] != '\0') evaluateABC(mock_eeprom); 
  
  // 4. Run the user's interactive command
  evaluateABC(userCode);
  
  long actualValue = vars[checkReg - 'a'].meta.value;
  if (actualValue == expectedValue) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    debugPrintf("       '%c' should be %ld, got %ld\r\n", checkReg, expectedValue, actualValue);
  }
}

void runStreamCodeTest(const char* testName, const char* streamInput, const char* code, char checkReg, long expectedValue) {
  testCount++;
  resetTestState();
  
  // Inject the mock human typing into the ring buffer
  rx_tail = 0; rx_head = 0;
  while(*streamInput) {
    rx_buffer[rx_head % 64] = *streamInput++;
    rx_head++;
  }
  
  // Initialize the terminal device 't' so the code can use it!
  vars['t'-'a'].meta.type = TYPE_DEV;
  vars['t'-'a'].meta.value = allocateDevice('T', 0);

  evaluateABC(code);
  
  long actualValue = vars[checkReg - 'a'].meta.value;
  if (actualValue == expectedValue) {
    debugPrintf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    debugPrintf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    debugPrintf("       '%c' should be %ld, got %ld\r\n", checkReg, expectedValue, actualValue);
  }
}

void runAllTests() {
  testCount = 0;
  passCount = 0;
  DEBUG_SERIAL.println("\r\n--- ABC Language Test Harness ---");

  // --- HARDWARE IN THE LOOP (HIL) TESTS ---
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  delay(10); 
  if (digitalRead(3) == HIGH) {
    DEBUG_SERIAL.println("[PASS] GP2 wired to GP3");
  } else {
    DEBUG_SERIAL.println("[FAIL] GP2 not wired to GP3!");
  }
  digitalWrite(2, LOW);

  evaluateABC("3P I\n");
  runTest("HIL: Active DigIn High Read", "2H\n1000 W\na:3I\n", 'a', 1);
  runTest("HIL: Active DigIn Low Read", "2L\n1000 W\nb:3I\n", 'b', 0);
  runAnalogTest("HIL: PWM/Analog 1", "22P:127\n1000000W\nc:26A\n", 'c', 512, 25);
  runAnalogTest("HIL: PWM/Analog 2", "22P:63\n1000000W\nc:26A\n", 'c', 256, 20);

  runTest(
    "Device: Digital I/O Handle Move", 
    "o:D('O',2,1)\n1000 W\ni:D('I',3,0)\na:i\n", 
    'a', 1
  );
  runTest(
    "Device: Digital Write Toggle", 
    "o:D('O',2,1)\ni:D('I',3,0)\no:0\n1000 W\nb:i\n", 
    'b', 0
  );
  runAnalogTest(
    "Device: PWM/Analog", 
    "o:D('PWM',22,127)\ni:D('A',26)\n500000 W\nc:i\n", 
    'c', 512, 20
  );
  runHardwareTest(
    "Device: RC Servo Target Stability", 
    "v:D('R',28,90)\nv:120\n", 
    28, 120, true
  );

  // --- INPUT MATCHER TESTS ---
  // 1. Exact Match
  setupMatchTest("hello");
  runMatchTest("Exact Match", "hello", true, 5);

  // 2. Early Mismatch (Leaves tail untouched)
  setupMatchTest("hi");
  runMatchTest("Early Mismatch", "hello", false, 0);

  // 3. Late Mismatch (Leaves tail untouched)
  setupMatchTest("hellx");
  runMatchTest("Late Mismatch", "hello", false, 0);

  // 4. Sequential Matches
  setupMatchTest("helloworld");
  runMatchTest("Sequential Match 1", "hello", true, 5);
  runMatchTest("Sequential Match 2", "world", true, 10);

  // 5. Ring Buffer Wrap-Around
  // Buffer size is 64. If we start at 62, "hello" writes to 62, 63, 0, 1, 2.
  setupMatchTest("hello", 62);
  runMatchTest("Ring Buffer Wrap-Around", "hello", true, 67);


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
  runHardwareTest("Servo Position", "28R:90\n", 28, 90, true);

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
  // D is the hardware init function. Allocating a standard PWM output. 
  // Since 't' defaults to Handle 1, the next allocated device will be Handle 2
  runTest("Hardware Function Call", "a:D('PWM', 3, 4)\n", 'a', 2);

  // TEST 16: Variable Parameter Shadowing
  // We manually spoof entering a subroutine with 2 arguments already on the stack.
  // 'a' should map to stack[0], and 'b' should map to stack[1]. 'c' maps to global.
  resetTestState(); // Use the formal reset to wipe type metadata
  testCount++;
  stack[0] = 77; // Spoof Arg 1
  stack[1] = 88; // Spoof Arg 2
  sp = 2;
  frame_pointer = 0;
  current_arg_count = 2;
  call_depth = 1; // SPOOF WE ARE INSIDE A FUNCTION
  scope_frame_pointer = 0;
  scope_arg_count = 2;

  // Try to copy 'a' to 'c'. If shadowing works, 'c' becomes 77, not 0
  evaluateABC("c:a\n"); 
  if (vars['c'-'a'].meta.value == 77) {
    debugPrintf("[PASS] Parameter Shadowing\r\n");
    passCount++;
  } else {
    debugPrintf("[FAIL] Parameter Shadowing\r\n");
  }
  call_depth = 0; // reset

  // TEST 17: Command Chaining with Semicolon
  // Assign 1 to a, 2 to b, then add them into c, all on one line.
  runTest("Semicolon Chaining", "a:1; b:2; c:a+b\n", 'c', 3);

  // TEST 18: Do-While Loop Mechanics
  // Initialize a to 0. Inside the loop, increment a by 1. Loop while a < 5.
  runTest("Loop Incrementing", "a:0\n[\na:a+1\na<5]\n", 'a', 5);

  // TEST 19: User Defined Functions & Parameter Shadowing
  // 1. Define 'a' as a function that adds its two arguments (which shadow 'a' and 'b').
  // 2. Call 'a(4, 5)' and assign the result to 'c'.
  runTest("Function Definition & Call", "a:\"a+b.\"\nc:a(4,5)\n", 'c', 9);

  // TEST 20: Quote Escaping and Nested Compilation
  // 'a' compiles a function into 'b' which sets 'c' to 1.
  runTest("Nested String Compilation", "a:\"b:\"\"c:1.\"\".\"\na()\nb()\n", 'c', 1);

  // TEST 21: Global Variable Access Inside Functions
  // 'c' is 3 globally. 'a' adds arg1(a) + arg2(b) + global(c). 4 + 5 + 3 = 12.
  runTest("Global Variable Scope", "c:3\na:\"a+b+c.\"\nc:a(4,5)\n", 'c', 12);

  // TEST 22: Nested Function Calls
  // 'a' increments its argument. 'b' passes its argument to 'a' and adds 10.
  runTest("Nested Function Calls", "a:\"a:a+1.\"\nb:\"b:a(a)+10.\"\nc:b(5)\n", 'c', 16);

  // TEST 23: Direct String Output
  // t is initialized as a Terminal ('T'). We assign a string literal to it.
  runStringTest("String Literal Output", "t:\"HELLO.\"\n", "HELLO.");

  // TEST 24: Base-10 Number Formatting
  // Assign 123 to 'a'. Format 'a' out to 't'.
  runStringTest("Base-10 Number Formatting", "a:123\nt%a\n", "123");

  // TEST 25: Radix-Aware Formatting (Hex)
  // Switch to base 16. Load 255 into 'g'. Format out to 't'.
  runStringTest("Radix Hex Formatting", "g:255\nr:16\nt%g\n", "ff");

  // TEST 26: Bytecode RAM Dumping
  // Assign a function to 'a'. Format 'a' out to 't' to dump its definition.
  runStringTest("Function Definition Dump", "a:\"2H.\"\nt%a\n", "\"2H.\"");

  // TEST 27: Basic Flash Device Write
  // Allocate 'F' to 'f'. Write "a:1" to it.
  runFlashTest("EEPROM Basic Write", "f:D('F')\nf:\"a:1.\"\n", "a:1.");

  // TEST 28: State Serialization (Dumping a variable)
  // Define a function in 'a'. Use % to dump its definition to 'f'.
  runFlashTest("EEPROM State Serialization", "a:\"2H.\"\nf:D('F')\nf:\"a:\"\nf%a\n", "a:\"2H.\"");

  // TEST 29: The Reboot Sequence
  // The boot script defines a function 'b' that returns 99.
  // We reboot, the boot script runs, and then the user calls 'c:b()'.
  runBootTest("EEPROM Boot Script Restoration", "b:\"99.\"\n", "c:b()\n", 'c', 99);

  // TEST 30: Basic Stream Match
  // Buffer has "hello". Code checks for "hello". If true, a=1. If false, a=2.
  runStreamCodeTest("VM Stream Exact Match", "hello", "a:2\nt=\"hello\"?a:1", 'a', 1);

  // TEST 31: Stream Mismatch Fallthrough
  // Buffer has "hi". Code checks for "hello". It fails early, falls through to else (a=2).
  runStreamCodeTest("VM Stream Early Mismatch", "hi", "t=\"hello\"?a:1!a:2", 'a', 2);

  // TEST 32: Sequential Multi-Match (The Golden Scenario)
  // Buffer has "hithere". 
  // Line 1 checks "hello" (fails, leaves buffer).
  // Line 2 checks "hi" (passes, consumes "hi", a=2).
  // Line 3 checks "there" (passes, consumes "there", b=3).
  runStreamCodeTest("VM Stream Sequential Matching", "hithere", 
                    "t=\"hello\"? a:1\n"
                    "t=\"hi\"? a:2\n"
                    "t=\"there\"? b:3\n", 
                    'b', 3);

  // TEST 33: Queue Length Read
  // Buffer has "abc". 'q' should dynamically evaluate to 3.
  runStreamCodeTest("Queue Length Read", "abc", "a:q\n", 'a', 3);

  // TEST 34: Queue Flush
  // Setting q:0 should instantly consume all characters in the ring buffer.
  runStreamCodeTest("Queue Flush", "abc", "q:0\na:q\n", 'a', 0);

  // TEST 35: Queue Pop
  // Decrementing q by 1 (q-1) should pop the oldest character, leaving 2.
  runStreamCodeTest("Queue Pop", "abc", "q-1\na:q\n", 'a', 2);

  // TEST 36: Stepper Motor Kinematics (Non-Blocking)
  // 1. Allocate Stepper 'm' to pins 3 & 4. Max Vel=1000, Accel=5000.
  // 2. Command move to 100.
  // 3. Wait 50ms (motor should be accelerating, >0 but <100). Save to 'a'.
  // 4. Wait 250ms (motor should easily reach 100 by now). Save to 'b'.
  // 5. If all kinematics checks pass, set c=1.
  #ifdef STEPPER_SUPPORT
  runTest(
    "Stepper", 
    "m:D('S', 3, 4, 1000, 5000)\n"
    "m:100\n"
    "50000 W\n"
    "a:m\n"
    "250000 W\n"
    "b:m\n"
    "c:0\n"
    "a>0? a<100? b=100? c:1\n"
    ,'c', 1);
  #else
  DEBUG_SERIAL.println("[SKIP] Stepper (no STEPPER_SUPPORT)");
  #endif // STEPPER_SUPPORT

  // TEST 37: Dynamixel Default Behaviors
  // Allocate to 'd'. Writing 500 implicitly hits address 116. 
  // Reading implicitly checks address 132 (which our mock auto-updates).
#ifdef DYNAMIXEL_SUPPORT
  runTest("Dynamixel Default Position Read/Write", 
          "d:D('D', 1, 115200)\n"
          "d:500\n"
          "a:d\n", 'a', 500);

  // TEST 38: Dynamixel Explicit Register Targeting (@)
  // Write 1 to LED register (65). Read from register 65 into 'b'.
  runTest("Dynamixel Explicit Register Target (@)", 
          "d:D('D', 1, 115200)\n"
          "65@d : 1\n"
          "b : 65@d\n", 'b', 1);
#else
  DEBUG_SERIAL.println("[SKIP] Dynamixel (no DYNAMIXEL_SUPPORT)");
#endif // DYNAMIXEL_SUPPORT

#ifdef ENCODER_SUPPORT
  DEBUG_SERIAL.println("[SKIP] Encoder (no test available)");
#else
  DEBUG_SERIAL.println("[SKIP] Encoder (no ENCODER_SUPPORT)");
#endif

  // TEST 39: I2C Multi-Byte Read & RAM Dereferencing
  #if defined(ARDUINO_ARCH_RP2040)
  Wire.setSDA(4); // Use a matching I2C0 pair!
  Wire.setSCL(5);
  #endif
  Wire.begin();
  
  Wire.beginTransmission(104); // DS1307 Address
  if (Wire.endTransmission() == 0) {
    Wire.end();
    runTest(
      "I2C Multi-Byte Read & Dereference", 
      "i:D('i', 5, 4, 104)\n" // SCL=5, SDA=4
      "r:16\n" // NOTE: registers a-f are now digits
      "0@i : 45\n"
      "2000 W\n"
      "1@i : 59\n"
      "2000 W\n"
      "x : 0 @ 2 i\n" // Use 'x' instead of 'a'
      "y : x @ 1\n"   // Use 'y' instead of 'b'
      "r:10\n", 
      'y', 89
    ); 
  } else {
    DEBUG_SERIAL.println("[SKIP] I2C Bus Target (Wokwi DS1307 at 0x68 not responding)");
  }

  // TEST 40: Fixed-Point Virtual Decimal Formatting
  // Assign 250 to 'a'. Format with Width 8, Precision 2. 
  // 4 leading spaces + "2.50" = 8 total characters.
  runStringTest(
    "Virtual Decimal Format (Padding)", 
    "a:250\nt:8%2a\n", 
    "    2.50"
  );

  // TEST 41: Fixed-Point Decimal Formatting (Negative)
  // Assign -250 to 'a'. Format with Width 8, Precision 2.
  // 3 leading spaces + "-2.50" = 8 total characters.
  runStringTest(
    "Virtual Decimal Format (Negative)", 
    "a:0-250\nt:8%2a\n", 
    "   -2.50"
  );

  // TEST 42: Fixed-Point Decimal Formatting (Leading Zeros)
  // Assign 5 to 'a'. Format with Width 0 (no padding), Precision 2.
  // The formatter should automatically insert the leading '0.0'
  runStringTest(
    "Virtual Decimal Format (Leading Zeros)", 
    "a:5\nt:0%2a\n", 
    "0.05"
  );

  debugPrintf("\r\n--- Test Run Complete: %d/%d Passed ---\r\n", passCount, testCount);
  resetTestState(true);
}

#endif