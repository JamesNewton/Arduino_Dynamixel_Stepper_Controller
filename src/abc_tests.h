// abc_tests.h
#ifndef ABC_TESTS_H
#define ABC_TESTS_H

// This file expects to be included at the bottom of main.cpp, 
// so it has native access to the VM globals and evaluateABC().

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
  code_ptr = 0; 
  quote_pending = false; 
  in_string_literal = false;
  in_char_literal = false;
  mock_out_ptr = 0;
  mock_out_buffer[0] = '\0';
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
    DEBUG_SERIAL.printf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    DEBUG_SERIAL.printf("       '%c' should be %ld, got %ld\r\n", checkReg, expectedValue, actualValue);
  }
}

void runAnalogTest(const char* testName, const char* code, char checkReg, long expectedValue, long tolerance) {
  testCount++;
  resetTestState();
  evaluateABC(code);
  long actualValue = vars[checkReg - 'a'].meta.value;
  if (abs(actualValue - expectedValue) <= tolerance) {
    DEBUG_SERIAL.printf("[PASS] %s (%ld, vs %ld+/-±%ld)\r\n", testName, actualValue, expectedValue, tolerance);
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    DEBUG_SERIAL.printf("       '%c' should be %ld (±%ld), got %ld\r\n", checkReg, expectedValue, tolerance, actualValue);
  }
}

void runHardwareTest(const char* testName, const char* code, int checkPin, long expectedValue, bool isServo = false) {
  testCount++;
  resetTestState();
  evaluateABC(code);

  long actualValue = isServo ? mock_servos[checkPin] : pin_shadow[checkPin];
  
  if (actualValue == expectedValue) {
    DEBUG_SERIAL.printf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n       Code: %s\r       Expected Pin %d to be %ld, but got %ld\r\n", testName, code, checkPin, expectedValue, actualValue);
  }
}

void runStringTest(const char* testName, const char* code, const char* expectedString) {
  testCount++;
  resetTestState();
  evaluateABC(code);
  
  if (strcmp(mock_out_buffer, expectedString) == 0) {
    DEBUG_SERIAL.printf("[PASS] %s\r\n", testName);
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] %s\r\n", testName);
    printCodeIndented(code);
    DEBUG_SERIAL.printf("       Expected output: '%s', but got '%s'\r\n", expectedString, mock_out_buffer);
  }
}

void runAllTests() {
  testCount = 0;
  passCount = 0;
  DEBUG_SERIAL.println("\r\n--- ABC Language Test Harness ---");

  runTest("Basic Numeric Assignment", "a:5\n", 'a', 5);
  runTest("Register Copy", "a:9\nb:a\n", 'b', 9);
  runTest("Simple Addition", "a:5\na:a+2\n", 'a', 7);
  runTest("Implicit Destination Math", "b:2\na:b+3\n", 'a', 5); 
  runTest("Conditional True", "a:1\na=1?b:9\n", 'b', 9);
  runTest("Conditional False", "a:0\na=1?b:9\n", 'b', 0);

  runHardwareTest("Digital Pin High", "13H\n", 13, 1);
  runHardwareTest("Digital Pin Low", "9L\n", 9, 0);
  runHardwareTest("Analog PWM Output", "5P:128\n", 5, 128);
  runHardwareTest("Servo Position", "2R:90\n", 2, 90, true);

  runTest("Radix Hex", "r:16\ng:10\n", 'g', 16);
  runTest("Radix Binary", "r:2\nb:101\n", 'b', 5);
  runTest("Single Quote ASCII", "a:'m'\n", 'a', 109);
  runTest("Stack Pointer Update", "10, 20,\n", 's', 2);
  runTest("Hardware Function Call", "a:D('M', 3, 4)\n", 'a', 1);

  // Variable Parameter Shadowing
  resetTestState(); 
  testCount++;
  stack[0] = 77; 
  stack[1] = 88; 
  sp = 2;
  frame_pointer = 0;
  current_arg_count = 2;
  call_depth = 1; 
  evaluateABC("c:a\n"); 
  if (vars['c'-'a'].meta.value == 77) {
    DEBUG_SERIAL.printf("[PASS] Variable Parameter Shadowing\r\n");
    passCount++;
  } else {
    DEBUG_SERIAL.printf("[FAIL] Variable Parameter Shadowing\r\n");
  }
  call_depth = 0; 

  runTest("Semicolon Chaining", "a:1; b:2; c:a+b\n", 'c', 3);
  runTest("Loop Incrementing", "a:0\n[\na:a+1\na<5]\n", 'a', 5);
  runTest("User Function Definition & Call", "a:\"a+b.\"\nc:a(4,5)\n", 'c', 9);
  runTest("Nested String Compilation", "a:\"b:\"\"c:1.\"\".\"\na()\nb()\n", 'c', 1);
  runTest("Global Variable Scope", "c:3\na:\"a+b+c.\"\nc:a(4,5)\n", 'c', 12);
  runTest("Nested Function Calls", "a:\"a:a+1.\"\nb:\"b:a(a)+10.\"\nc:b(5)\n", 'c', 16);

  runStringTest("String Literal Output", "t:D('T')\nt:\"HELLO.\"\n", "HELLO.");
  runStringTest("Base-10 Number Formatting", "t:D('T')\na:123\nt%a\n", "123");
  runStringTest("Radix Hex Formatting", "t:D('T')\ng:255\nr:16\nt%g\n", "ff");
  runStringTest("Function Definition Dump", "t:D('T')\na:\"2H.\"\nt%a\n", "\"2H.\"");

  // --- HARDWARE IN THE LOOP (HIL) TESTS ---
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

  evaluateABC("3P I\n");
  runTest("HIL: Active DigIn High Read", "2H\n1000 W\na:3I\n", 'a', 1);
  runTest("HIL: Active DigIn Low Read", "2L\n1000 W\nb:3I\n", 'b', 0);
  runAnalogTest("HIL: PWM/Analog Filter 1", "22P:128\n500000W\nc:26A\n", 'c', 512, 25);
  runAnalogTest("HIL: PWM/Analog Filter 2", "22P:64\n500000W\nc:26A\n", 'c', 256, 20);

  runTest("Device: Digital I/O Handle Move", "o:D('O',2,1)\n1000 W\ni:D('I',3,0)\na:i\n", 'a', 1);
  runTest("Device: Digital Write Toggle", "o:D('O',2,1)\ni:D('I',3,0)\no:0\n1000 W\nb:i\n", 'b', 0);
  runAnalogTest("Device: PWM/Analog Filter", "o:D('P',22,127)\ni:D('A',26)\n500000 W\nc:i\n", 'c', 512, 20);
  runHardwareTest("Device: RC Servo Target Stability", "v:D('R',2,90)\nv:120\n", 2, 120, true);

  DEBUG_SERIAL.printf("\r\n--- Test Run Complete: %d/%d Passed ---\r\n", passCount, testCount);
}

#endif