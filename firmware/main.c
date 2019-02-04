#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "softuart.h"
#include "nmea.h"

#define PIN_MOSI PB0
#define PIN_SCK PB2
#define PIN_LOAD PB3
#define PIN_LIGHT_SENSE PB4

static const uint8_t kNumDigits = 6;

static DateTime _gpsTime = {0, 0, 0, 0, 0, 0};

inline void setup_pins()
{
    // Load/CS pin is active low - initialise as high
    PORTB = _BV(PIN_LOAD);

    // MAX7219 pins as output
    // Soft UART and LDR are inputs by omission
    DDRB = _BV(PIN_MOSI) | _BV(PIN_SCK) | _BV(PIN_LOAD);
}

/**
 * Clock out a byte to the MAX7219 (SPI-like)
 */
static void spi_send(uint8_t byte)
{
    // Clock out 8 bits, MSB first
    for (uint8_t i = 8; i != 0; --i) {
        PORTB &= ~_BV(PIN_SCK);

        if (byte & 0x80) {
            PORTB |= _BV(PIN_MOSI);
        } else {
            PORTB &= ~_BV(PIN_MOSI);
        }

        // Clock out
        PORTB |= _BV(PIN_SCK);

        // Next bit
        byte <<= 1;
    }
}

/**
 * Write to a register on the MAX7219
 */
static void max7219_cmd(uint8_t address, uint8_t data)
{
    // Select chip (active low)
    PORTB &= ~_BV(PIN_LOAD);

    spi_send(address);
    spi_send(data);

    // Pull chip select high to latch data
    PORTB |= _BV(PIN_LOAD);
}

static void max7219_init()
{
    // Initialise all digits to be zero
    for (uint8_t i = 0; i < kNumDigits; ++i) {
        max7219_cmd(i + 1, 0xFF);
    }

    // Set scan mode to 6 digits
    max7219_cmd(0x0B, kNumDigits);

    // Disable test mode
    max7219_cmd(0x0F, 0);

    // Enable binary decode mode
    max7219_cmd(0x09, 0xFF);

    // Set brightness
    max7219_cmd(0x0A, 0x0C);

    // Enable display
    max7219_cmd(0x0C, 1);
}

/**
 * Send the current time to the MAX7219 as 6 BCD digits
 */
static inline void display_update()
{
    // Adjust for NZ timezone (hardcoded for now)
    _gpsTime.hour += 13;
    if (_gpsTime.hour > 23) {
        _gpsTime.hour -= 24;
    }

    // Send time to display
    uint8_t digit = 1;

    for (int8_t i = 0; i < 3; ++i) {

        // Manually digit into tens and ones columns
        // This saves 25 bytes vs. using the divide and modulo operators.
        uint8_t ones = ((uint8_t*) &_gpsTime)[i];
        uint8_t tens = 0;

        while (ones >= 10) {
            ones -= 10;
            ++tens;
        }

        max7219_cmd(digit++, tens);
        max7219_cmd(digit++, ones);
    }
}

static inline void display_no_signal()
{
    static uint8_t waitIndicator = 0;

    for (uint8_t i = 0; i < kNumDigits; ++i) {
        if (waitIndicator == i) {
            // Light decimal point
            max7219_cmd(i + 1, 0x8F);
        } else {
            // Blank digit
            max7219_cmd(i + 1, 0x7F);
        }
    }

    ++waitIndicator;
}

int main(void)
{
    setup_pins();
    max7219_init();

    display_no_signal();

    while (true) {
        // Wait for a line of text from the GPS unit
        GpsReadStatus status = gps_read_time(&_gpsTime);

        switch (status) {
            case kGPS_Success:
                // Update the display with the new parsed time
                display_update();
                break;

            case kGPS_NoMatch:
                // Ignore partial and unknown sentences
                break;

            case kGPS_NoSignal:
                // Walk the decimal point across the display to indicate activity
                display_no_signal();
                break;

            case kGPS_InvalidChecksum:
                max7219_cmd(1, 11 /* E */);
                max7219_cmd(2, 1);

                // Blank other digits
                for (uint8_t i = 2; i < kNumDigits; ++i) {
                    max7219_cmd(i+1, 0x7F);
                }
                break;

            case kGPS_BadFormat:
                // This state is returned if the UART line isn't pulled high (ie. GPS unplugged)
                max7219_cmd(1, 11 /* E */);
                max7219_cmd(2, 2);

                // Blank other digits
                for (uint8_t i = 2; i < kNumDigits; ++i) {
                    max7219_cmd(i+1, 0x7F);
                }
                break;
        }
    }
}

