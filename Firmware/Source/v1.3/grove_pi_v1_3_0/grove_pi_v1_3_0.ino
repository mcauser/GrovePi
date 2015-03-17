#include <Wire.h>           // For handling I2C
#include "MMA7660.h"        // For 3-Axis Digital Accelerometer(±1.5g)
#include "DS1307.h"         // For RTC
#include "DHT.h"            // For Temperature and Humidity Sensor
#include "Grove_LED_bar.h"  // For LED Bar
#include "TM1637.h"         // For 4-Digit Display
#include "ChainableLED.h"   // For Chainable RGB LED

MMA7660 acc;
DS1307 clock;
DHT dht;
Grove_LED_Bar ledbar[6];  // 7 instances for D2-D8, however, max 4 bars, you can't use adjacent sockets, 4 pin display
TM1637 fourdigit[6];      // 7 instances for D2-D8, however, max 4 displays, you can't use adjacent sockets, 4 pin display
ChainableLED rgbled[6];   // 7 instances for D2-D8

#define GROVEPI_ADDRESS 0x04

#define debug 0             // Make 0 to disable debugging

// Firmware version (major.minor.patch)
#define version_major 1
#define version_minor 3
#define version_patch 0


// Command list received from the Raspberry Pi
#define digital_read_cmd      1  // Digital read - Read the value from a digital pin, either HIGH or LOW
#define digital_write_cmd     2  // Digital write - Write a HIGH or LOW to a digital pin
#define analog_read_cmd       3  // Analog read - Read the value from an analog pin
#define analog_write_cmd      4  // Analog write - Writes an analog value (PWM wave) to a pin
#define pin_mode_cmd          5  // Pin mode - Configure a pin to behave either as input or output
#define firmware_version_cmd  8  // Firmware version - Get the firmware version

// Grove Ultrasonic Ranger
// http://seeedstudio.com/wiki/Grove_-_Ultrasonic_Ranger
#define ultrasonic_read_cmd  7  // Ultrasonic read - Get the distance in cm

// Grove 3-Axis Digital Accelerometer(±1.5g)
// http://seeedstudio.com/wiki/Grove_-_3-Axis_Digital_Accelerometer%28%C2%B11.5g%29
#define accelerometer_read_cmd  20  // Accelerometer read - Get X, Y and Z from the 1.5g accelerometer

// Grove RTC
// http://seeedstudio.com/wiki/Grove_-_RTC
#define rtc_read_cmd  30  // RTC read - Get the time and date from the RTC

// Grove Temperature and Humidity Sensor Pro
// http://seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor_Pro
// http://seeedstudio.com/wiki/Grove_-_Temperature_and_Humidity_Sensor
#define dht_read_cmd  40  // DHT read - Read the temperature and humidity
  #define is_DHT11    0   // DHT11
  #define is_DHT22    1   // DHT22
  #define is_DHT21    2   // DHT21
  #define is_AM2301   3   // AM2301

// Grove LED Bar
// http://seeedstudio.com/wiki/Grove_-_LED_Bar
#define led_bar_init_cmd               50  // LED bar init - Initialise a LED bar
#define led_bar_orientation_cmd        51  // LED bar orientation - Set LED bar orientation
#define led_bar_set_level_cmd          52  // LED bar set level - Set LED bar level (0-10)
#define led_bar_set_single_led_cmd     53  // LED bar set single LED - Set a single LED on the LED bar
#define led_bar_toggle_single_led_cmd  54  // LED bar toggle single LED - Toggle a single LED on the LED bar
#define led_bar_set_state_cmd          55  // LED bar set state - Set all LEDs on the LED bar
#define led_bar_get_state_cmd          56  // LED bar get state - Get the current state of the LEDs on the LED bar

// Grove 4 Digit Display (7 segment)
// http://seeedstudio.com/wiki/Grove_-_4-Digit_Display
#define four_digit_init_cmd                         70  // 4 digit init - Initialise a 4 digit display
#define four_digit_set_brightness_cmd               71  // 4 digit set brightness - Set brightness (0-7)
#define four_digit_value_without_leading_zeros_cmd  72  // 4 digit value without leading zeros - Right aligned decimal value without leading zeros
#define four_digit_value_with_leading_zeros_cmd     73  // 4 digit value with leading zeros - Right aligned decimal value with leading zeros
#define four_digit_set_individual_digit_cmd         74  // 4 digit set individual digit - Display a number in one of the 4 segments
#define four_digit_set_individual_segment_cmd       75  // 4 digit set individual segment - Set individual LEDs in one of the 4 segments
#define four_digit_set_scoreboard_cmd               76  // 4 digit set scoreboard - Set left and right numbers (0-99) with a colon
#define four_digit_display_analog_read_cmd          77  // 4 digit display analog read - Display analog read for n seconds, 4 samples per second
#define four_digit_on_cmd                           78  // 4 digit display on - Turn the entire display on
#define four_digit_off_cmd                          79  // 4 digit display off - Turn the entire display off

// Grove Chainable RGB LED
// http://seeedstudio.com/wiki/Grove_-_Chainable_RGB_LED
#define store_rgb_color_cmd                      90  // Store RGB color - Store a color for later use
#define chainable_rgb_init_cmd                   91  // Chainable RGB init - Initialise a chain of one or more RGB LEDs
#define chainable_rgb_test_pattern_cmd           92  // Chainable RGB test pattern - Set all LEDs to white, red, green, blue, cyan, magenta, yellow or black using a combination of 3 RGB bits
  #define pattern_just_this_led                  0   // this led only
  #define pattern_all_other_leds                 1   // all leds except this led
  #define pattern_this_led_and_inwards           2   // this led and all leds inwards
  #define pattern_this_led_and_outwards          3   // this led and all leds outwards
#define chainable_rgb_set_leds_with_pattern_cmd  93  // Chainable RGB set LEDs with pattern - Set color using pattern: 0 this LED only, 1: all except this, 2: this and all inwards, 3: this and all outwards
#define chainable_rgb_set_leds_with_modulo_cmd   94  // Chainable RGB set LEDs with modulo - Set color on all LEDs >= offset when mod remainder is 0
#define chainable_rgb_set_level_cmd              95  // Chainable RGB set level - Set color on all LEDs <= level, outwards unless reverse


//I2C Message variables
int cmd[5];  // command bytes
int index = 0;  // number of command bytes read from wire, once 5 command bytes have been read, the command is executed
int flag = 0;
byte payload[9];  // return variables

//int i;  // not used
//byte float_array[4];  // not used
int aRead = 0;
byte accFlag = 0;  // for lazy init()
byte clkFlag = 0;  // for lazy init()
int8_t accv[3];  // accel x,y,z
byte rgb[] = { 0, 0, 0 };  // stored color
//int pin;  // not used
//int j;  // not used

void setup()
{
  // Enable the serial port on the GrovePi
  if (debug) {
    Serial.begin(9600);
    Serial.print("Ready");
  }

  // Setup I2C
  Wire.begin(GROVEPI_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop()
{
  if (index == 4 && flag == 0) {
    flag = 1;

    // Digital Read
    // cmd: [1, pin, unused, unused]
    if (cmd[0] == digital_read_cmd) {
      pinMode(cmd[1], INPUT);
      payload[1] = digitalRead(cmd[1]);
    }

    // Digital Write
    // cmd: [2, pin, value, unused]
    //   value: 0 = LOW, 1 = HIGH
    if (cmd[0] == digital_write_cmd) {
      pinMode(cmd[1], OUTPUT);
      digitalWrite(cmd[1], cmd[2]);
    }

    // Analog Read
    // cmd: [3, pin, unused, unused]
    if (cmd[0] == analog_read_cmd) {
      aRead = analogRead(cmd[1]);
      payload[1] = aRead / 256;
      payload[2] = aRead % 256;
    }

    // Analog Write
    // cmd: [4, pin, value, unused]
    //   value: 0-255
    if (cmd[0] == analog_write_cmd) {
      analogWrite(cmd[1], cmd[2]);
    }

    // Set up pinMode
    // cmd: [5, pin, pin mode, unused]
    //   pin mode: INPUT, OUTPUT
    if (cmd[0] == pin_mode_cmd) {
      pinMode(cmd[1], cmd[2]);
    }

    // Grove Ultrasonic Ranger - Ultrasonic Read
    // cmd: [7, pin, unused, unused]
    if (cmd[0] == ultrasonic_read_cmd) {
      pinMode(cmd[1], OUTPUT);
      digitalWrite(cmd[1], LOW);
      delayMicroseconds(2);
      digitalWrite(cmd[1], HIGH);
      delayMicroseconds(5);
      digitalWrite(cmd[1], LOW);
      pinMode(cmd[1], INPUT);
      long duration = pulseIn(cmd[1], HIGH);
      long RangeCm = duration / 29 / 2;
      payload[1] = RangeCm / 256;
      payload[2] = RangeCm % 256;
      if (debug) {
        Serial.println('Ultrasonic Read');
        Serial.println(payload[1]);
        Serial.println(payload[2]);
      }
    }

    // Firmware version
    // cmd: [8, unused, unused, unused]
    if (cmd[0] == firmware_version_cmd) {
      payload[1] = version_major;
      payload[2] = version_minor;
      payload[3] = version_patch;
    }

    // Grove 3-Axis Digital Accelerometer(±1.5g) - Read X,Y,Z
    // Uses MMA7660FC accelerometer
    // cmd: [20, pin, unused, unused]
    if (cmd[0] == accelerometer_read_cmd) {
      if (accFlag == 0) {
        acc.init();
        accFlag = 1;
      }
      acc.getXYZ(&accv[0], &accv[1], &accv[2]);
      payload[1] = accv[0];
      payload[2] = accv[1];
      payload[3] = accv[2];
    }

    // RTC read
    // cmd: [30, pin, unused, unused]
    if (cmd[0] == rtc_read_cmd) {
      if (clkFlag == 0) {
        clock.begin();
        //Set time the first time
        //clock.fillByYMD(2013,1,19);
        //clock.fillByHMS(15,28,30);//15:28 30"
        //clock.fillDayOfWeek(SAT);//Saturday
        //clock.setTime();//write time to the RTC chip
        clkFlag = 1;
      }
      clock.getTime();
      payload[1] = clock.hour;
      payload[2] = clock.minute;
      payload[3] = clock.second;
      payload[4] = clock.month;
      payload[5] = clock.dayOfMonth;
      payload[6] = clock.year;
      payload[7] = clock.dayOfMonth;
      payload[8] = clock.dayOfWeek;
    }

    // Temperature and Humidity Read
    // cmd: [40, pin, dht type, unused]
    //   dht type: 0 = DHT11, 1 = DHT22, 2 = DHT21, 3 = AM2301
    if (cmd[0] == dht_read_cmd) {
        if (cmd[2] == is_DHT11) {
        dht.begin(cmd[1], DHT11);
      }
      else if (cmd[2] == is_DHT22) {
        dht.begin(cmd[1], DHT22);
      }
      else if (cmd[2] == is_DHT21) {
        dht.begin(cmd[1], DHT21);
      }
      else if (cmd[2] == is_AM2301) {
        dht.begin(cmd[1], AM2301);
      }
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (debug) {
        Serial.println('DHT Read');
        Serial.print(t);
        Serial.print("#");
      }
      byte *b1 = (byte*)&t;
      byte *b2 = (byte*)&h;
      for (int j = 0; j < 4; j++) {
        payload[j+1] = b1[j];
      }
      for (int j = 4; j < 8; j++) {
        payload[j+1] = b2[j-4];
      }
    }

    // Grove LED Bar - Initialise
    // cmd: [50, pin, orientation, unused]
    //   orientation: 0 = red to green (normal), 1 = green to red (reverse)
    if (cmd[0] == led_bar_init_cmd) {
      // clock pin is always next to the data pin
      ledbar[cmd[1]-2].begin(cmd[1]+1, cmd[1], cmd[2]); // clock, data, orientation
    }

    // Grove LED Bar - Change the orientation
    // Green to red, or red to green
    // cmd: [51, pin, orientation, unused]
    //   orientation: 0 = red to green (normal), 1 = green to red (reverse)
    if (cmd[0] == led_bar_orientation_cmd && ledbar[cmd[1]-2].ready()) {
      ledbar[cmd[1]-2].setGreenToRed(cmd[2]);
    }

    // Grove LED Bar - Set level (0-10)
    // cmd: [52, pin, level, unused]
    //   level: 0 = all leds off, 10 = all leds on
    if (cmd[0] == led_bar_set_level_cmd && ledbar[cmd[1]-2].ready()) {
      ledbar[cmd[1]-2].setLevel(cmd[2]);
    }

    // Grove LED Bar - Set a single led
    // cmd: [53, pin, led, state]
    //   led: 1-10, 1 = red led when in normal mode, otherwise when in reverse 10 is the red led
    //   state: 0 = off, 1 = on
    if (cmd[0] == led_bar_set_single_led_cmd && ledbar[cmd[1]-2].ready()) {
      ledbar[cmd[1]-2].setLed(cmd[2], cmd[3]);
    }

    // Grove LED Bar - Toggle a single led
    // Inverts the specified led
    // cmd: [54, pin, led, unused]
    //   led: 1-10, 1 = red led when in normal mode, otherwise when in reverse 10 is the red led
    if (cmd[0] == led_bar_toggle_single_led_cmd && ledbar[cmd[1]-2].ready()) {
      ledbar[cmd[1]-2].toggleLed(cmd[2]);
    }

    // Grove LED Bar - Set the current state
    // One bit for each led, across two bytes
    // 0    = 0x0   = 0b000000000000000 = all leds off
    // 5    = 0x05  = 0b000000000000101 = leds 1 and 3 on, all others off
    // 341  = 0x155 = 0b000000101010101 = leds 1,3,5,7,9 on, 2,4,6,8,10 off
    // 1023 = 0x3ff = 0b000001111111111 = all leds on
    //                       |        |
    //                       10       1
    // cmd: [55, pin, bits 1-8, bits 9-10]
    //   bits 1-8: lsb = led 1
    //   bits 9-10: lsb = led 9, only first two bits used in this byte, the rest ignored
    if (cmd[0] == led_bar_set_state_cmd && ledbar[cmd[1]-2].ready()) {
      ledbar[cmd[1]-2].setBits(cmd[2] ^ (cmd[3] << 8));
    }

    // Grove LED Bar - Get the current state
    // cmd: [56, pin, unused, unused]
    if (cmd[0] == led_bar_get_state_cmd && ledbar[cmd[1]-2].ready()) {
      unsigned int state = ledbar[cmd[1]-2].getBits();
      payload[1] = state & 0xFF;
      payload[2] = state >> 8;
    }

    // Grove 4 Digit Display - Initialise
    // cmd: [70, pin, unused, unused]
    if (cmd[0] == four_digit_init_cmd) {
      // clock pin is always next to the data pin
      fourdigit[cmd[1]-2].begin(cmd[1], cmd[1]+1);  // clock, data
    }

    // Grove 4 Digit Display - Set brightness
    // cmd: [71, pin, brightness, unused]
    //   brightness: 0-7, 0 = dim, 7 = bright
    if (cmd[0] == four_digit_set_brightness_cmd && fourdigit[cmd[1]-2].ready()) {
      fourdigit[cmd[1]-2].setBrightness(cmd[2]);  // setBrightness(brightness)
    }

    // Grove 4 Digit Display - Show right aligned decimal value without leading zeros
    // Can output [  : 0] through [FF:FF], 4 bits per 7 segment - segments containing leading zeros are switched off
    // To output 99:99, you would need to split the number 9999 into two bytes (0b00001111, 0b01110111)
    // cmd: [72, pin, bits 1-8, bits 9-16]
    //   bits 1-8: least significant bits
    //   bits 9-16: most significant bits
    if (cmd[0] == four_digit_value_without_leading_zeros_cmd && fourdigit[cmd[1]-2].ready()) {
      fourdigit[cmd[1]-2].showNumberDec(cmd[2] ^ (cmd[3] << 8), false);  // showNumberDec(number, leading_zero)
    }

    // Grove 4 Digit Display - Show right aligned decimal value with leading zeros
    // Can output [00:00] through [FF:FF], 4 bits per 7 segment
    // To output 99:99, you would need to split the number 9999 into two bytes (0b00001111, 0b01110111)
    // cmd: [73, pin, bits 1-8, bits 9-16]
    //   bits 1-8: least significant bits
    //   bits 9-16: most significant bits
    if (cmd[0] == four_digit_value_with_leading_zeros_cmd && fourdigit[cmd[1]-2].ready()) {
      fourdigit[cmd[1]-2].showNumberDec(cmd[2] ^ (cmd[3] << 8), true);  // showNumberDec(number, leading_zero)
    }

    // Grove 4 Digit Display - Set individual digit
    // Display a 0-9 number (dec) in one of the 4 segments (index)
    // cmd: [74, pin, index, dec]
    //   index: 0-3, 0 = left segment, 3 = right segment
    //   dec: 0-9, the number to display in the segment
    if (cmd[0] == four_digit_set_individual_digit_cmd && fourdigit[cmd[1]-2].ready()) {
      uint8_t data[] = {};
      data[0] = fourdigit[cmd[1]-2].encodeDigit(cmd[3]);  // encodeDigit(number)
      fourdigit[cmd[1]-2].setSegments(data, 1, cmd[2]);   // setSegments(segments[], length, position)
    }

    // Grove 4 Digit Display - Set individual segment
    // Set the individual leds that make up a segment to make your own custom characters
    // cmd: [75, pin, index, binary]
    //   index: 0-3, 0 = left segment, 3 = right segment
    //   binary: representation of the 8 bits that make up the 7 segment (8th bit is the decimal place, only exposed on 2nd segment)
    if (cmd[0] == four_digit_set_individual_segment_cmd && fourdigit[cmd[1]-2].ready()) {
      // 0xFF = 0b11111111 = Colon,G,F,E,D,C,B,A
      // Colon only works on 2nd segment (index 1)
      //     -A-
      //  F |   | B
      //     -G-
      //  E |   | C
      //     -D-
      uint8_t data[] = {};
      data[0] = cmd[3];  // byte
      fourdigit[cmd[1]-2].setSegments(data, 1, cmd[2]);  // setSegments(segments[], length, position)
    }

    // Grove 4 Digit Display - Set left and right with colon separator
    // You could use this to display time, eg. 12:59 or a 2 player scoreboard 05:07
    // The separator is always illuminated and leading zeros displayed
    // cmd: [76, pin, left, right]
    //   left: 0-99, value for the first two segments
    //   right: 0-99, value for the next two segments
    if (cmd[0] == four_digit_set_scoreboard_cmd && fourdigit[cmd[1]-2].ready()) {
      uint8_t data[] = {};
      // 1st segment
      data[0] = fourdigit[cmd[1]-2].encodeDigit(cmd[2] / 10);  // encodeDigit(number)
      // 2nd segment
      data[1] = fourdigit[cmd[1]-2].encodeDigit(cmd[2] % 10);  // encodeDigit(number)
      // colon
      data[1] |= 0x80;
      // 3rd segment
      data[2] = fourdigit[cmd[1]-2].encodeDigit(cmd[3] / 10);  // encodeDigit(number)
      // 4th segment
      data[3] = fourdigit[cmd[1]-2].encodeDigit(cmd[3] % 10);  // encodeDigit(number)
      // send
      fourdigit[cmd[1]-2].setSegments(data, 4, 0);  // setSegments(segments[], length, position)
    }

    // Grove 4 Digit Display - Analog read
    // Analog read (0-1023) another pin and output it's value for n seconds, 4 samples per second
    // cmd: [77, pin, analog pin, seconds]
    //   analog pin: which analog pin to read from
    //   seconds: sample the analog pin for this many seconds, outputting each value to the display
    if (cmd[0] == four_digit_display_analog_read_cmd && fourdigit[cmd[1]-2].ready()) {
      int pin = cmd[2];
      int reads = 4 * cmd[3];  // 1000/250 * cmd[3]

      // reading analog pin 4x per second
      for (int i = 0; i < reads; i++) {
        fourdigit[cmd[1]-2].showNumberDec(analogRead(pin), false);  // showNumberDec(number, leading_zero)
        delay(250);
      }
    }

    // Grove 4 Digit Display - Display on
    // cmd: [78, pin, unused, unused]
    if (cmd[0] == four_digit_on_cmd && fourdigit[cmd[1]-2].ready()) {
      uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF };
      fourdigit[cmd[1]-2].setSegments(data, 4, 0);  // setSegments(segments[], length, position)
    }

    // Grove 4 Digit Display - Display off
    // cmd: [79, pin, unused, unused]
    if (cmd[0] == four_digit_off_cmd && fourdigit[cmd[1]-2].ready()) {
      uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 };
      fourdigit[cmd[1]-2].setSegments(data, 4, 0);  // setSegments(segments[], length, position)
    }

    // Grove Chainable RGB LED - Store RGB color for later use
    // cmd: [90, red, green, blue]
    //   red: 0-255
    //   green: 0-255
    //   blue: 0-255
    if (cmd[0] == store_rgb_color_cmd) {
      rgb[0] = cmd[1];
      rgb[1] = cmd[2];
      rgb[2] = cmd[3];
    }

    // Grove Chainable RGB LED - Initialise
    // Initialise a chain of leds
    // cmd: [91, pin, num leds, unused]
    //   num leds: how many LED modules do you have connected in series?
    if (cmd[0] == chainable_rgb_init_cmd) {
      rgbled[cmd[1]-2].begin(cmd[1], cmd[1]+1, cmd[2]);  // clock, data, num leds
    }

    // Grove Chainable RGB LED - Test pattern
    // Initialise a chain of leds and set all to a test color
    // cmd: [92, pin, num leds, color code]
    //   num leds: how many LED modules do you have connected in series?
    //   color code: 0 black (off), 1 blue, 2 green, 3 cyan, 4 red, 5 magenta, 6 yellow, 7 white
    if (cmd[0] == chainable_rgb_test_pattern_cmd) {
      rgbled[cmd[1]-2].begin(cmd[1], cmd[1]+1, cmd[2]);

      // figure out which color to display, a single bit for each rgb led (0b00RRGGBB)
      byte rr = ((cmd[3] & 4) >> 2) * 255,
           gg = ((cmd[3] & 2) >> 1) * 255,
           bb = ((cmd[3] & 1)) * 255;

      // set each led to the specified color
      for (int i = 0; i < cmd[2]; i++) {
        rgbled[cmd[1]-2].setColorRGB(i, rr, gg, bb);
      }
    }

    // Grove Chainable RGB LED - Set LEDs with pattern
    // Set one or more leds to the stored color using pattern
    // cmd: [93, pin, pattern, which led]
    //   pattern: 0 = this led only, 1 all leds except this led, 2 this led and all leds inwards, 3 this led and all leds outwards
    //   which led: 0 = led closest to the GrovePi, 1 = second led counting outwards
    if (cmd[0] == chainable_rgb_set_leds_with_pattern_cmd) {
      if (cmd[2] == pattern_just_this_led) {
        // pattern 0: set an individual led to the stored color
        rgbled[cmd[1]-2].setColorRGB(cmd[3], rgb[0], rgb[1], rgb[2]);  // which led, red, green, blue
      }
      else {
        // set all leds to stored color
        byte num_leds = rgbled[cmd[1]-2].getNumLeds();

        for (int i = 0; i < num_leds; i++)
        {
          // pattern 1: set all leds other than this one to the stored color
          // pattern 2: this led and all previous leds, inwards
          // pattern 3: this led and all next leds, outwards
          if ((cmd[2] == pattern_all_other_leds && i != cmd[3]) || (cmd[2] == pattern_this_led_and_inwards && i <= cmd[3]) || (cmd[2] == pattern_this_led_and_outwards && i >= cmd[3])) {
            rgbled[cmd[1]-2].setColorRGB(i, rgb[0], rgb[1], rgb[2]);  // which led, red, green, blue
          }
        }
      }
    }

    // Grove Chainable RGB LED - Set LEDs with modulo
    // Set one or more leds to the stored color using modulo
    // cmd: [94, pin, led offset, modulo divisor]
    //   led offset: 0 = led closest to the GrovePi, counting outwards
    //   modulo divisor: when 1 (default) sets stored color on all leds >= offset, when 2 sets every 2nd led >= offset and so on
    if (cmd[0] == chainable_rgb_set_leds_with_modulo_cmd) {
      // modulo divisor must be >= 1
      if(cmd[3] < 1) {
        cmd[3] = 1;
      }

      // get the chain length
      byte num_leds = rgbled[cmd[1]-2].getNumLeds();

      // starting at the offset, step through each led and if the result of the modulo operator results in zero, set the stored color on the led
      for (int i = cmd[2]; i < num_leds; i++) {
        // use modulo to set every n led
        if ((i - cmd[2]) % cmd[3] == 0) {
          rgbled[cmd[1]-2].setColorRGB(i, rgb[0], rgb[1], rgb[2]);  // which led, red, green, blue
        }
      }
    }

    // Grove Chainable RGB LED - Set level (0 to num leds), reversible
    // cmd: [95, pin, level, reverse]
    //   level: 0 to number of leds in the chain counting outwards from the GrovePi, 0 = all off, 1 = first led
    //   reverse: 0-1, when 1 counts inwards from the most furthest led in the chain
    if (cmd[0] == chainable_rgb_set_level_cmd) {
      // get the chain length
      byte num_leds = rgbled[cmd[1]-2].getNumLeds();

      if (cmd[3] == 0) {
        // outwards
        for (int i = 0; i < num_leds; i++) {
          if (cmd[2] > i) {
            rgbled[cmd[1]-2].setColorRGB(i, rgb[0], rgb[1], rgb[2]);  // which led, red, green, blue
          }
          else {
            rgbled[cmd[1]-2].setColorRGB(i, 0, 0, 0);  // which led, red, green, blue
          }
        }
      }
      else {
        // inwards
        for (int i = num_leds; i > 0; i--) {
          if ((num_leds - cmd[2]) <= i) {
            rgbled[cmd[1]-2].setColorRGB(i, rgb[0], rgb[1], rgb[2]);  // which led, red, green, blue
          }
          else {
            rgbled[cmd[1]-2].setColorRGB(i, 0, 0, 0);  // which led, red, green, blue
          }
        }
      }
    }
  }
}

// Receive commands via I2C
void receiveData(int byteCount) {
  while (Wire.available()) {
    // When the buffer gets filled up with 4 bytes (all commands are 4 bytes in size), set the index back to 0 to read the next command
    if (Wire.available() == 4) {
      flag = 0;
      index = 0;
    }
    // Load the command byte into the buffer
    cmd[index++] = Wire.read();
  }
}

// callback for sending data
void sendData()
{
  if (cmd[0] == digital_read_cmd) {
    Wire.write(payload[1]);
  }
  if (cmd[0] == analog_read_cmd || cmd[0] == ultrasonic_read_cmd || cmd[0] == led_bar_get_state_cmd) {
    Wire.write(payload, 3);
  }
  if (cmd[0] == firmware_version_cmd || cmd[0] == accelerometer_read_cmd) {
    Wire.write(payload, 4);
  }
  if (cmd[0] == rtc_read_cmd || cmd[0] == dht_read_cmd) {
    Wire.write(payload, 9);
  }
}
