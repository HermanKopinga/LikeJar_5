#include <avr/eeprom.h>

// Like button by Herman Kopinga herman@kopinga.nl
// Works together with a homebrew Arduino on an Atmega8
// Soldered on a perfboard with a Kingbright 3 digit 7 segment common cathode LED module.

// Version history:
// 0.92: Beta for 5th prototype (button pin changed to hardware interrupt pin).
// 0.91: Beta for 2nd prototype.
// 0.9: Beta version removed test code & redundant variables and improved documentation.

// Software based on:
// Arduino 7 segment display example software
// http://www.hacktronics.com/Tutorials/arduino-and-7-segment-led.html
 
// License: none whatsoever.
 
// Global variables


// for 3 digit display in the 'like jar' prototype 2
//  A4. 
// A   A
// 3   2
//  A5. 
// 9   1
// 9   2
//  10. 11
const byte sevenSegDigits[11][7] =   { { 0,0,0,0,0,0,1 },  // = 0
                                     { 1,0,0,1,1,1,1 },  // = 1
                                     { 0,0,1,0,0,1,0 },  // = 2
                                     { 0,0,0,0,1,1,0 },  // = 3
                                     { 1,0,0,1,1,0,0 },  // = 4
                                     { 0,1,0,0,1,0,0 },  // = 5
                                     { 0,1,0,0,0,0,0 },  // = 6
                                     { 0,0,0,1,1,1,1 },  // = 7
                                     { 0,0,0,0,0,0,0 },  // = 8
                                     { 0,0,0,0,1,0,0 },  // = 9
                                     { 1,1,1,1,1,1,1 }   // = space
                                     };
//5(11), 7(13), a2(25), a4(27) a5(28), 6(12), a1(24)... a3(26)
//2(4),3(5),4(6)

const byte sevenSegPins[8] = {A4,A2,12,10,9,A3,A5,11};

const int digit1pin = 13;
const int digit2pin = A0;
const int digit3pin = A1;
const int buttonpin = 2;
          
long ticks = 0;
const byte reset = 6 ;
const byte divide = 1;
int likes;                    // The number of button presses.
int num;
int value;                    // The number displayed on the LEDs.
int n = 0;
int buttonState = 0;          // Current state of the button.
int lastButtonState = 0;      // Previous state of the button.
long coolDownTicks = 0;       // Hold delay before a new button press is registered.
                                 
void setup()
{
  //Set all the LED pins as output.
  for (byte pinCount = 0; pinCount < 9; pinCount++)
  {
     pinMode(sevenSegPins[pinCount], OUTPUT);
  }

  // Common pins for the 3 digits.
  pinMode(digit1pin, OUTPUT);
  pinMode(digit2pin, OUTPUT);
  pinMode(digit3pin, OUTPUT);

  // Setup specific for prototype 2.
  // Pullup resistor on button pin
  pinMode(buttonpin, INPUT);
  digitalWrite(buttonpin, HIGH);       // turn on pullup resistor

  // Apparently it's important to enable the pullup on all unused pins.
  // Saved about 1 mA using this.
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  
  writeDot(1);          // start with the "dot" off

  // Read the number of Likes from last run.
  eeprom_read_block((void*)&likes, (void*)0, sizeof(likes));
  
//  Serial.begin(115200);
}

void writeDot(byte dot) 
{
  digitalWrite(sevenSegPins[7], dot);
}

// Function that writes the passed digit to the output pins. 
// Depending on which cathode is grounded another digit is lit.
void sevenSegWrite(byte digit, byte dot) 
{
  for (byte loopCount = 0; loopCount < 8; ++loopCount) {
    digitalWrite(sevenSegPins[loopCount], sevenSegDigits[digit][loopCount]);
  }
  writeDot(dot);
}

void loop()
{
  // Always increase the ticks counter.
  ticks++;
  
  // Decrease the cooldown if it is currently running.
  if (coolDownTicks > 0)
  {
    coolDownTicks--;
  }
  else
  {
    ////////////// 
    //Manage the button.
    
    // Read current button state.
    buttonState = digitalRead(buttonpin);
  
    // compare the buttonState to its previous state
    if (buttonState != lastButtonState) {
      if (buttonState == LOW) {
        // if the current state is LOW then the button was pushed.
        if (coolDownTicks == 0)
        {
          likes++; 
          coolDownTicks = 2000;
          eeprom_write_block((const void*)&likes, (void*)0, sizeof(likes));
        }
        else
        {
          // Cooldown wasn't done, restart cooldown as 'punishment' for pressing to soon.
          coolDownTicks = 2000;
        }
      }
    }
    // save the current state as the last state, 
    // for next time through the loop
    lastButtonState = buttonState;
  }
  
  ////////////// 
  // (Re)Write the digits

  // Write a digit at a time for 20 ticks.
  n = ticks/divide;
  
  // Most significant digit
  if(n == 1)
  {
    digitalWrite(digit2pin,0);
    digitalWrite(digit3pin,0);
    sevenSegWrite(value % 1000 / 100, 1);
    digitalWrite(digit1pin,1);

  }

  // Middle significant digit  
  if(n == 2)
  {
    digitalWrite(digit1pin,0);
    digitalWrite(digit3pin,0);
    sevenSegWrite(value % 100 / 10, 1);
    digitalWrite(digit2pin,1);

  }

  // Least significant digit  
  if(n == 3)
  {
    digitalWrite(digit1pin,0);
    digitalWrite(digit2pin,0);
    if (coolDownTicks > 0)
    {
      sevenSegWrite(value % 10, 0);
    }
    else
    {
      sevenSegWrite(value % 10, 1);
    }
    digitalWrite(digit3pin,1);
  } 
  
  // The rest of the time the leds are off, saves 10 mA power.
  if(n > 3)
  {
     digitalWrite(digit1pin,0);
     digitalWrite(digit2pin,0);
     digitalWrite(digit3pin,0);     
  }
  
  // Ticks won't be bigger than reset.
  if(ticks > reset) 
  {
    ticks = 0;
    num++;
  }
  
  // Every (10 * ticks) loops the value we work with is updated.
  if(num > 10)
  {
    num = 0;
    value = likes;
  }
}
