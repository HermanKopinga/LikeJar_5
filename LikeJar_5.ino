#include <avr/eeprom.h>
#include <avr/sleep.h>

// Like button by Herman Kopinga herman@kopinga.nl
// Works together with a homebrew Arduino on an Atmega8.

// Version history:
// 0.98: Added switch between regular and Brightlight mode. Updated comments, finally resolved the License.
// 0.97: Thousands now show on every boot for 2 seconds. Added bright mode (more power, more light).
// 0.96: Added Thousands
// 0.95: Added check to ignore the button press that wakes the Atmega.
// 0.94: Added basic avr Sleep untill interrupt. Removed Narcoleptic_m8, didn't work.
// 0.93: Added Narcoleptic_m8 library and sleep function.
// 0.92: Beta for 5th version (button pin changed to hardware interrupt pin).
// 0.91: Beta for 2nd prototype.
// 0.9: Beta version removed test code & redundant variables and improved documentation.

// Stands on the soulders of:

/* Arduino 7 segment display example software
   http://www.hacktronics.com/Tutorials/arduino-and-7-segment-led.html */

/* Sleep Demo Serial
 * Copyright (C) 2006 MacSimski 2006-12-30
 * Copyright (C) 2007 D. Cuartielles 2007-07-08 - Mexico DF 
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 */
 
/* As a result the license for this project is GNU General Public License version 3.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. */
 
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
byte reset = 6;
byte resetWant = 6;
const byte divide = 1;
int coolDownTicksSet = 2000;
long likes;                     // The number of button presses.
int num;
int value;                    // The number displayed on the LEDs.
int n = 0;
int buttonState = 0;          // Current state of the Like button.
unsigned long significantStateMillis = 0;
int moduloExtra = 1;          // To display more significant digits. (thousands)
int lastButtonState = 0;      // Previous state of the button.
long coolDownTicks = 0;       // Hold delay before a new button press is registered.
long dimTimer = 0;            // Dual timer, for dimming display and starting sleep mode.
int ignorePress = 0;          // Ignore the button press when coming out of sleep.
int moduloIndicator = 1;

int dig1Min = 1;
int dig1Max = 1;
int dig2Min = 3;
int dig2Max = 3;
int dig3Min = 5;
int dig3Max = 5;

static int vccRead (byte us =250) {
  analogRead(6);    // set up "almost" the proper ADC readout
  bitSet(ADMUX, 3); // then fix it to switch to channel 14
  delayMicroseconds(us); // delay substantially improves accuracy
  bitSet(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;
  word x = ADC;
  return x ? (1100L * 1023) / x : -1;
}
                                 
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
  
  // Start with all 'dots' off.
  digitalWrite(sevenSegPins[7], 1);

  // Read the number of Likes from last run.
  eeprom_read_block((void*)&likes, (void*)0, sizeof(likes));
  
  // Serial is only used for debugging.
  Serial.begin(9600);
  Serial.println(vccRead());
}

void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}

void sleepNow()         // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we
   * choose the according
   * sleep mode: SLEEP_MODE_PWR_DOWN
   *
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
   * accidentally pushed interrupt button doesn't interrupt
   * our running program. if you want to be able to run
   * interrupt code besides the sleep function, place it in
   * setup() for example.
   *
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
   *
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */

  attachInterrupt(0,wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  attachInterrupt(1,wakeUpNow, LOW); // use interrupt 1 (pin 3) and run function
  // wakeUpNow when pin 2 gets LOW

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.

}

// Function that writes the passed digit to the output pins. 
// Depending on which cathode is grounded another digit is lit.
void sevenSegWrite(byte digit, byte dot) 
{
  for (byte loopCount = 0; loopCount < 8; ++loopCount) {
    digitalWrite(sevenSegPins[loopCount], sevenSegDigits[digit][loopCount]);
  }

  digitalWrite(sevenSegPins[7], dot);
}

void loop()
{
  // Always increase the ticks counter.
  ticks++;
  dimTimer++;
  
  // Decrease the cooldown if it is currently running.
  if (coolDownTicks > 0)
  {
    coolDownTicks--;
  }
  
  ////////////// 
  //Manage the buttons.
  
  // Read current button state.
  buttonState = digitalRead(buttonpin);
  
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // If the current state is LOW then the button was pushed.
    // This construct makes sure only one button press is registered despite the loop running more often.
    if (buttonState == LOW) {

      // Reset Like counter if pin 6 is pulled low and the like button is pressed.
      if (digitalRead(6) == LOW)
      {
        likes = 0;
        ignorePress = 1;
        eeprom_write_block((const void*)&likes, (void*)0, sizeof(likes));
      }

      // Switch on/off Brightlight mode if pin 5 is pulled low and the like button is pressed.
      // Brightlight mode makes the display brighter by not turning it off.
      if (digitalRead(5) == LOW)
      {
        // Ignore this button press, it's not a like, its functional.
        ignorePress = 1;

        // Determine the current state of Brightlight mode.
        if (resetWant != 9)
        {
          dig1Min = 0;
          dig1Max = 3;
          dig2Min = 4;
          dig2Max = 6;
          dig3Min = 7;
          dig3Max = 9;
          reset = resetWant = 9;
        }
        else
        {
          dig1Min = 1;
          dig1Max = 1;
          dig2Min = 3;
          dig2Max = 3;
          dig3Min = 5;
          dig3Max = 5;
          reset = resetWant = 6;
        }
      }
      
      // Determine if this button press was a like, if so register it.
      if (ignorePress == 1)
      {
        ignorePress = 0;
      }
      else if (coolDownTicks == 0)
      {
        // Another Like, JAY!
        likes++; 
        // Set the cooldown timer, no likes will be 
        coolDownTicks = coolDownTicksSet;
        // Write the new like value to the eeprom so it is saved even is power is lost.
        eeprom_write_block((const void*)&likes, (void*)0, sizeof(likes));
      }
      else
      {
        // Cooldown wasn't done, restart (and increase) cooldown as 'punishment' for pressing too soon.
        coolDownTicks = coolDownTicksSet * 1,5;
      }

      // There was an interaction, reset the sleep timer.
      dimTimer = 0;
      
      // There was an interaction, reset the dim value.
      reset = resetWant;
    }
  }
  
  // save the current state as the last state, 
  // for next time through the loop
  lastButtonState = buttonState;

  // The timer that sleeps the device.
  // 30000 ticks is about 26 seconds.
  // 2076921 is about 30 minutes.
  if (dimTimer > 2076921)
  //if (timer > 10000)
  {
    reset = 20;
  }

  // 8307692 is about 2 hours.
  if (dimTimer > 8307692)
  //if (timer > 30000)
  {
    ignorePress = 1;
    digitalWrite(digit1pin,0);
    digitalWrite(digit3pin,0);
    digitalWrite(digit2pin,0);  
    sleepNow();     // sleep function called here
    dimTimer = 0;
  }
  
  ////////////// 
  // (Re)Write the digits

  // Write a digit at a time for 20 ticks.
  n = ticks/divide;
  
  if (digitalRead(5) == LOW)
  {
    significantStateMillis = millis();
  }
  
  if (significantStateMillis + 2000 > millis())
  {
    moduloExtra = 1000;
    moduloIndicator = 0;
  }
  else
  {
    moduloExtra = 1;
    moduloIndicator = 1;
  }
  
  // Most significant digit
  if(n >= dig1Min && n <= dig1Max)
  {
    digitalWrite(digit2pin,0);
    digitalWrite(digit3pin,0);
    sevenSegWrite(value % (1000*moduloExtra) / (100*moduloExtra), 1);
    digitalWrite(digit1pin,1);
  }

  // Middle significant digit  
  else if(n >= dig2Min && n <= dig2Max)
  {
    digitalWrite(digit1pin,0);
    digitalWrite(digit3pin,0);
    sevenSegWrite(value % (100*moduloExtra) / (10*moduloExtra), moduloIndicator);
    digitalWrite(digit2pin,1);
  }

  // Least significant digit  
  else if(n >= dig3Min && n <= dig3Max)
  {
    digitalWrite(digit1pin,0);
    digitalWrite(digit2pin,0);
    if (coolDownTicks > 0)
    {
      sevenSegWrite(value % (10*moduloExtra) / moduloExtra, 0);
    }
    else
    {
      sevenSegWrite(value % (10*moduloExtra) / moduloExtra, 1);
    }
    digitalWrite(digit3pin,1);
  } 
  
  // The rest of the time the leds are off, saves 10 mA power.
  else
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
