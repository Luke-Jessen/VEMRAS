/*!
 * @file Adafruit_LEDBackpack.h
 *
 * Part of Adafruit's Arduino library for our I2C LED Backpacks:
 * ----> http://www.adafruit.com/products/
 * ----> http://www.adafruit.com/products/
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface. There are multiple selectable I2C addresses. For backpacks
 * with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
 * with 3 Address Select pins: 0x70 thru 0x77
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text above must be included in any redistribution
 */

#ifndef Adafruit_LEDBackpack_h
#define Adafruit_LEDBackpack_h
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define LED_ON 1  ///< GFX color of lit LED segments (single-color displays)
#define LED_OFF 0 ///< GFX color of unlit LED segments (single-color displays)

#define LED_RED 1    ///< GFX color for red LED segments (bi-color displays)
#define LED_YELLOW 2 ///< GFX color for yellow LED segments (bi-color displays)
#define LED_GREEN 3  ///< GFX color for green LED segments (bi-color displays)

#define HT16K33_BLINK_CMD 0x80       ///< I2C register for BLINK setting
#define HT16K33_BLINK_DISPLAYON 0x01 ///< I2C value for steady on
#define HT16K33_BLINK_OFF 0          ///< I2C value for steady off
#define HT16K33_BLINK_2HZ 1          ///< I2C value for 2 Hz blink
#define HT16K33_BLINK_1HZ 2          ///< I2C value for 1 Hz blink
#define HT16K33_BLINK_HALFHZ 3       ///< I2C value for 0.5 Hz blink

#define HT16K33_CMD_BRIGHTNESS 0xE0 ///< I2C register for BRIGHTNESS setting



/*
Segment names for 14-segment alphanumeric displays.
See https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing/usage

    -------A-------
    |\     |     /|
    | \    J    / |
    |   H  |  K   |
    F    \ | /    B
    |     \|/     |
    |--G1--|--G2--|
    |     /|\     |
    E    / | \    C
    |   L  |   N  |
    | /    M    \ |
    |/     |     \|
    -------D-------  DP
*/

#define ALPHANUM_SEG_A 0b0000000000000001  ///< Alphanumeric segment A
#define ALPHANUM_SEG_B 0b0000000000000010  ///< Alphanumeric segment B
#define ALPHANUM_SEG_C 0b0000000000000100  ///< Alphanumeric segment C
#define ALPHANUM_SEG_D 0b0000000000001000  ///< Alphanumeric segment D
#define ALPHANUM_SEG_E 0b0000000000010000  ///< Alphanumeric segment E
#define ALPHANUM_SEG_F 0b0000000000100000  ///< Alphanumeric segment F
#define ALPHANUM_SEG_G1 0b0000000001000000 ///< Alphanumeric segment G1
#define ALPHANUM_SEG_G2 0b0000000010000000 ///< Alphanumeric segment G2
#define ALPHANUM_SEG_H 0b0000000100000000  ///< Alphanumeric segment H
#define ALPHANUM_SEG_J 0b0000001000000000  ///< Alphanumeric segment J
#define ALPHANUM_SEG_K 0b0000010000000000  ///< Alphanumeric segment K
#define ALPHANUM_SEG_L 0b0000100000000000  ///< Alphanumeric segment L
#define ALPHANUM_SEG_M 0b0001000000000000  ///< Alphanumeric segment M
#define ALPHANUM_SEG_N 0b0010000000000000  ///< Alphanumeric segment N
#define ALPHANUM_SEG_DP 0b0100000000000000 ///< Alphanumeric segment DP




  /*!
    @brief  Start I2C and initialize display state (blink off, full
            brightness).
    @param  _addr  I2C address.
    @param  theWire  TwoWire bus reference to use.
    @return  true if successful, otherwise false

  */
  void begin(uint8_t _addr);

  /*!
    @brief  Turn display on or off
    @param  state  State: true = on, false = off
  */
  void setDisplayState(bool state, uint8_t address);

  /*!
    @brief  Set display brightness.
    @param  b  Brightness: 0 (min) to 15 (max).
  */
  void setBrightness(uint8_t b, uint8_t address);

  /*!
    @brief  Set display blink rate.
    @param  b  One of:
               HT16K33_BLINK_OFF       = no blinking
               HT16K33_BLINK_2HZ       = 2 Hz blink
               HT16K33_BLINK_1HZ       = 1 Hz blink
               HT16K33_BLINK_HALFHZ    = 0.5 Hz blink
  */
  void blinkRate(uint8_t b, uint8_t address);

  /*!
    @brief  Issue buffered data in RAM to display.
  */
  void writeDisplay();

  /*!
    @brief  Clear display.
  */
  void clear(void);

  extern uint16_t displaybuffer[16]; ///< Raw display data






  /*!
    @brief  Set specific digit # to a numeric value.
    @param  x    Character position.
    @param  num  Numeric (not ASCII) value.
    @param  dot  If true, light corresponding decimal.
  */
  void writeDigitNum(uint8_t x, uint8_t num, bool dot);



  /*!
    @brief  Set or unset colon segment.
    @param  state  'true' to enable colon, 'false' for off.
  */
  void drawColon(bool state);

  /*!
    @brief  General integer-printing function used by some of the print()
            variants.
    @param  n     Numeric value.
    @param  base  Base (2 = binary).
  */
  void printNumber(long n, uint8_t base);

  /*!
    @brief  General float-printing function used by some of the print()
            variants.
    @param  n           Numeric value.
    @param  fracDigits  Fractional-part digits.
    @param  base        Base (default DEC = base 10).
  */
  void printFloat(double n, uint8_t fracDigits, uint8_t base);

  /*!
    @brief  Light display segments in an error-indicating configuration.
  */
  void printError(void);

  /*!
    @brief  Issue colon-on directly to display (bypass buffer).
  */






  /*!
    @brief  Write single character of alphanumeric display as raw bits
            (not a general print function).
    @param  n        Character index (0-3).
    @param  bitmask  Segment bitmask.
  */
  void writeDigitRaw(uint8_t n, uint16_t bitmask);

  /*!
    @brief  Write single ASCII character to alphanumeric display.
    @param  n      Character index (0-3).
    @param  ascii  ASCII character.
    @param  dot    If true, also light corresponding dot segment.
  */
  void writeDigitAscii(uint8_t n, uint8_t ascii, bool dot);

  void writeString(uint8_t start, char *string, uint8_t length);


#endif // Adafruit_LEDBackpack_h
