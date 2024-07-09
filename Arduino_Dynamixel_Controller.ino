#include <EEPROM.h>
//unsigned char EEPROM[100]; 
//The stupid EEPROM library being included causes all errors to show up as "invalid header file"

//https://docs.arduino.cc/learn/built-in-libraries/eeprom/
//EEPROM.length(); //returns the length of the 
//byte = EEPROM[addr]; //read ? from EEPROM
//EEPROM[addr] = bytes; //write
//byte = EEPROM.read(addr); //return a single byte out of EEPROM from address
//EEPROM.write(addr, byte); //write a byte in to EEPROM at address
//EEPROM.update(addr, byte); //write a byte in to EEPROM at address, but only if it's not already there
//EEPROM.get(addr, var); //get variable of any size out of EEPROM from address
//EEPROM.put(addr, var); //put variable of any size in to EEPROM at address

#define BAUD 115200
//#define DYNAMIXEL_SUPPORT
//Note: If enabled, the Dynamixel Arduino shield is required and the standard Arduino UNO serial port is NOT functional! 
//You must install the USB to Serial interface on the Dynamixel Shield and use that for communication. RTFM
//#define STEPPER_SUPPORT

#define PROG_SPACE 100
//count of pins which support analog modes
#define ANALOG_PINS NUM_ANALOG_INPUTS
//Turns out NUM_DIGITAL_PINS counts ALL pins which can be used digitally, 
//including the analog pins
#define NUM_PINS NUM_DIGITAL_PINS
//Track the digital /only/ pins.
#define DIGITAL_PINS (NUM_DIGITAL_PINS - ANALOG_PINS)
//The first analog pin number is the last digital only pin + 1
//change above if you want to use fewer than actual pins.
#define CYCLE_DELAY 100
#define DXL_DIR_PIN -1
#define DEBUG_SERIAL Serial

long n; //number accumulator. New digits are shifted in. 'a'-'z' 
long p; //secondary accumulator. ',' shifts n into p.
/* n and p can reference 
- just numbers, e.g. degrees for a servo, number of steps to move, etc...
- a pin number, addressing that pin, if the value is between 1 and NUM_PINS
- a register address, between NUM_PINS and that plus NO_REGS
- an address in EEPROM, between NUM_PINS+NO_REGS and that plus EEPROM.length()

Assuming 20 digital pins, 'a' becomes 20, 'b' 21, etc.. 
Digits are shifted into n and p, so '1a' becomes 21 same as 'b'
EEPROM is addressed after 'z' via numbers. '1z' is the first EEPROM address

*/
long d;
long radix;
int sign;
char cmd; //the current command byte code
char op; //the current operation
enum Q {quote_off, quote_on, quote_end } quoting; //track in quotes, just ended, or well past.
bool debugging;
#define NO_REGS 26
long reg[NO_REGS]; //registers, each is a long, several bytes
int pc; //program counter. Indexes EEPROM
int stack[5]; //stack
int sp; //stack pointer
bool skipping; //are we skipping for a conditional

#define REGLTR(addr) (reg[addr-'a'])
//make it easy to index the reg array with a character letter e.g.REGLTR('b') is reg[1]
byte *mem; //will point into reg



void delayus(unsigned long us) {
  if (us>10000) { //can't delayMicroseconds() more than 16838
    delay(us/10000);
    us=us % 10000;
    }
  delayMicroseconds(us);
  }


void setup() {
  DEBUG_SERIAL.begin(BAUD);
  while(!DEBUG_SERIAL); //Wait until the serial port is opened
  int l = EEPROM.length();
  if (l>PROG_SPACE) l = PROG_SPACE;
  EEPROM.get(0,n);//load up the count of memory used
  if (n<=0 || n>l) { 
    for (int i = 0; i<l; i++) {
      EEPROM[i] = 0;
      }
    n = sizeof(reg[0]);
    }
  REGLTR('m') = n;
  REGLTR('z') = l;
  printreg('m'-'a');

  n=0; //number
  p=0; //pin number
  d=2; //delay. Default is 2uS or 250KHz
  radix=10;
  sign=1;
  //mem = reg[0];
  op = 0;
  pc = 0;
  sp = 0;
  debugging = true;
  DEBUG_SERIAL.print(NUM_PINS);
  DEBUG_SERIAL.print(" pins, ");
  DEBUG_SERIAL.print(DIGITAL_PINS);
  DEBUG_SERIAL.print(" digital, ");
  DEBUG_SERIAL.print(ANALOG_PINS);
  DEBUG_SERIAL.print(" analog. ");
  }

int printstat() {
  DEBUG_SERIAL.print("\n");
  DEBUG_SERIAL.print(p);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(n);

  DEBUG_SERIAL.print(" R:");
  for (int i=0; i < 3; i++) {
	  DEBUG_SERIAL.print(reg[i]);
	  DEBUG_SERIAL.print(", ");
  }

  DEBUG_SERIAL.print(" S:");
  for (int i=0; i < 5; i++) {
	  DEBUG_SERIAL.print(stack[i]);
	  DEBUG_SERIAL.print(", ");
  }

  DEBUG_SERIAL.print(" E:");
  for (int i=sizeof(reg[0]); i < REGLTR('m'); i++) {
	  DEBUG_SERIAL.print((char)EEPROM[i]);
  }
  DEBUG_SERIAL.print(" #");
  DEBUG_SERIAL.println(REGLTR('m'));
  return 1;
}

void printreg(int n) {
  DEBUG_SERIAL.print((char)('a'+n));
  DEBUG_SERIAL.print(":");
  DEBUG_SERIAL.print(reg[n]);
  DEBUG_SERIAL.print("\"");
  char c;
  for (int i=reg[n]; i < REGLTR('m'); i++) {
    c = (char)EEPROM[i];
	DEBUG_SERIAL.print(c);
    if ('.' == c) break;
  }
  DEBUG_SERIAL.print("\"\n");
}

void display_status() {
  DEBUG_SERIAL.print("{"); //optional, just to signal start of data
  if (0==n) { //if we didn't have a number selecting a pin
    DEBUG_SERIAL.print("\"?\":[\""); //optional, just to signal start of data
    for (int p = 0; p < NUM_PINS; p++) { //get all the pins
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
    DEBUG_SERIAL.print("]");
    }
  else { //specific pin or address
    DEBUG_SERIAL.print("\"");
    DEBUG_SERIAL.print(n);
    DEBUG_SERIAL.print("\":[");
    if (DXL_DIR_PIN != n) { //don't mess with the servo pins
      int value = 0;
      if (NUM_PINS > n) { //it's a pin
        value = digitalRead(n);
      } else { //it's an address
        if (NO_REGS+NUM_PINS < n) n = NO_REGS+NUM_PINS; 
        value = reg[n-NUM_PINS]; //to a register
      }
      DEBUG_SERIAL.print(value); //just that one value
      if (n > DIGITAL_PINS && n < NUM_PINS) { //if there is an analog channel
        value = analogRead(n); //use that as well.
        DEBUG_SERIAL.print(",");
        DEBUG_SERIAL.print(value); //also return it
        }
      }
    DEBUG_SERIAL.print("]");
    }
  DEBUG_SERIAL.println("}");
  //DEBUG_SERIAL.write(04); //EOT
  //sending EOT can help OS serial device drivers return data instead of waiting forever for the file to end.
  //https://stackoverflow.com/questions/50178789/signal-end-of-file-in-serial-communication
  DEBUG_SERIAL.print("\n"); //new line can help after EOT to tigger xmit on OS serial handler


}

/* test code

a"13L_b@?13H.".
b,1!
a) //verify led on
b,0!
a) //verify led off

a"13L_2I?13H.".
//set pin 2 to 0
a) //verify no led
//set pin 2 to 1
a) //verify led on

*/
void loop(){
  //use this while loop so we can "continue" from anywhere when we finish with a new character
  while ((DEBUG_SERIAL.available()) //if data has arrived, 
         || (pc > 0)//or we are running from EEPROM
        ) { 
    int c = pc?(char)EEPROM.read(pc++):DEBUG_SERIAL.read(); //get the next opcode
    cmd = char(c); 
    if (debugging) DEBUG_SERIAL.print(cmd);
    if ('\"'==cmd) { //quote
      if (quoting == quote_off) {//this is a new quote
        //if (NUM_PINS >= n || REG_SIZE+NUM_PINS <= n) {DEBUG_SERIAL.print("reg?");}
        reg[n-NUM_PINS]=REGLTR('m'); //reg being defined points to mem at reg m
        quoting = quote_on;
        continue;
      }
      if (quoting == quote_on) { //end of a quote?
        EEPROM.put(EEPROM.length()-sizeof(reg[0]), REGLTR('m')); //save the number of bytes we've used
        quoting = quote_end;
        if (debugging) printreg(n-NUM_PINS);
        continue;
      }
      if (quoting == quote_end) { //quote JUST ended
        quoting = quote_on; //this was a double double, restart
        } //go on and save the quote character.
    } else if (quoting == quote_end) { //not a quote, and we just ended one
        quoting = quote_off; //so we are really done now, it couldn't be a double double
        } //keep going
    if (quoting == quote_on) { //if we are quoting, just put it away
      EEPROM.update(REGLTR('m'),cmd); //add the current address, then increment it
      REGLTR('m')++;
      continue; //and do nothing more
    }
    if ('\n'==cmd || '\r'==cmd || '_'==cmd || '.'==cmd || 0==cmd) { //EOL
      if (NUM_PINS < p && NO_REGS+NUM_PINS > p) { //p is a register
        reg[p-NUM_PINS]=n; //TODO check for an ':' op as well. actually do a switch here.
        p=0; //for now
      }
      if ('.'==cmd || 0==cmd) {
        pc = stack[sp]; //return, always a zero at stack[0]
        if (sp) sp--; //stop at 0
        }
      n=0; cmd=0;//clear command and value (but not p) at end of line.
      d=0; //delay doesn't last past one line
      skipping = false; //only skip to the end of the line.
      DEBUG_SERIAL.print("cr");
      continue; //loop now, no delay
    }
	if (skipping) continue; //unless it was an EOL, it's after a failed '?', so we don't care. 
	if ('0' <= c && c <= '9') { //if it's a digit
      n = (c-'0')*sign + n*radix; //add it to n, shift n up 1 digit
      sign = 1; //in case it was negative
      continue; //and loop
      }
    if ('a' <= cmd && cmd <= 'z') { //if it's a variable
      n = n + (c - 'a') + NUM_PINS; //store the address
      //note that n is folded in. e.g. 1a is b
      continue; //and loop
    }
    if (' '==cmd || '\t'==cmd) { continue;} //whitespace does nothing
    if (','==cmd) { p=n; n=0; continue;} //save n to p, clear n, loop
    if (0==n) {
      if ('-'==cmd) {sign = -1; continue;} //if we don't have a number, it's a negative
      /* this seems like a super bad idea... why do this?
      n=p; //if we don't have a value, use the prevous pin number. 
      //Note this means n can't be zero unless p is. 1,0 isn't possible, it becomes 1,1
      */
      } 
    switch (cmd) {
      case ')': //call
        if (sp >= sizeof(stack)/sizeof(stack[0])) {
          DEBUG_SERIAL.println("\nSTACK OVERFLOW");
          break;
          }
        sp++; stack[sp] = pc;
        pc = reg[n-NUM_PINS];
        n = 0;
        if (debugging) {DEBUG_SERIAL.print("run from "); DEBUG_SERIAL.println(pc);}
        break;
      case '@': // //value AT address, source based on range
        if (n > DIGITAL_PINS && n < NUM_PINS) { n = analogRead(n); }
        else if (n < DIGITAL_PINS) { n = digitalRead(n); }
        else if (n < NUM_PINS+NO_REGS ) { n = reg[n-NUM_PINS]; }
        else if (n >= NUM_PINS+NO_REGS ) { n = EEPROM[n-(NUM_PINS+NO_REGS)];}
        break;
      case '!': // Write value to address, destination depends on range
        if (p > DIGITAL_PINS && p < NUM_PINS) { analogWrite(p, n); }
        else if (p < DIGITAL_PINS) { digitalWrite(p, n); }
        else if (p < NUM_PINS+NO_REGS ) { reg[p-NUM_PINS] = n; }
        else if (p >= NUM_PINS+NO_REGS ) { EEPROM[p-(NUM_PINS+NO_REGS)]=n;}
        break;
      case '?': //get information
      	if (pc) { //executing
          if (!n) {//and not true
            skipping = true;
          	}
          n = 0; //done with the condition, clear for the next command.
          }
        else { //interactive
	        display_status();
        	}
        break;
      case 'H': //set pin n output high
        pinMode(n,OUTPUT);
        digitalWrite(n,HIGH);
        break;
      case 'L': //set pin n output low
        pinMode(n,OUTPUT);
        digitalWrite(n,LOW);
        break;
      case 'I': //set pin n input
        pinMode(n,INPUT);
      	if (n>DIGITAL_PINS) n = analogRead(n);
      	else n = digitalRead(n);
        break;
      case 'P': //set pin n input with pullup
        pinMode(n,INPUT_PULLUP);
      	if (n>DIGITAL_PINS) n = analogRead(n);
      	else n = digitalRead(n);
        break;
      case 'A': //set pin p to analog output value n
        pinMode(p,OUTPUT);
        analogWrite(p,n);
        break;
      case 'D': //delay n ms per instruction
        d=n;
        break;


      default:
        DEBUG_SERIAL.print("\"");
        DEBUG_SERIAL.print(n);
        DEBUG_SERIAL.print(cmd);
        DEBUG_SERIAL.println("?\"");
      } 
    //n=0; //zero out value for next command. No, only at end of line.
    //delayus(d); //wait a bit for the next cycle.
    //p is NOT cleared, so you can keep sending new commands only
    cmd=0; //done with command.
	}
  //delayus(CYCLE_DELAY); //save a little energy if we don't have characters. 
  if (debugging) {
    printstat();
    while (!DEBUG_SERIAL.available()) ;
  }


}
 
