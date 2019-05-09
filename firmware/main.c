#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

#include "softuart.h"
#include "nmea.h"

#define PIN_MOSI PB0
#define PIN_SCK PB2
#define PIN_LOAD PB3
#define PIN_LIGHT_SENSE PB4

static const uint8_t kNumDigits = 6;

static int8_t _timezoneOffset = 0;
static DateTime _gpsTime = {0, 0, 0, 0, 0, 0};

static inline void setup_pins()
{
    // Load/CS pin is active low - initialise as high
    PORTB = _BV(PIN_LOAD);

    // MAX7219 pins as output
    // Soft UART and LDR are inputs by omission
    DDRB = _BV(PIN_MOSI) | _BV(PIN_SCK) | _BV(PIN_LOAD);
}

static inline void setup_adc()
{
    // Select PB4 (PIN_LIGHT_SENSE) in the ADC multiplexer
    // Put the significant 8-bits in the upper register as we only want to read that
    ADMUX = _BV(MUX1) | _BV(ADLAR);

    // Enable ADC conversions
    ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE);
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

/**
 * Configure the MAX7219 for use
 */
static void max7219_init()
{
    // Set scan mode to 6 digits
    max7219_cmd(0x0B, kNumDigits);

    // Disable test mode
    max7219_cmd(0x0F, 0);

    // Enable binary decode mode
    max7219_cmd(0x09, 0xFF);

    // Enable display
    max7219_cmd(0x0C, 1);
}

/**
 * Send the current time to the MAX7219 as 6 BCD digits
 */
static void display_update()
{
    // Adjust hour for timezone
    int8_t hour = _gpsTime.hour;
    hour += _timezoneOffset;

    if (hour > 23) {
        hour -= 24;
    } else if (hour < 0) {
        hour += 24;
    }

    _gpsTime.hour = hour;
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

static void display_no_signal()
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
    if (waitIndicator == kNumDigits) {
        waitIndicator = 0;
    }
}

static void display_error_code(uint8_t code)
{
    max7219_cmd(1, 11 /* E */);
    max7219_cmd(2, code);

    // Blank other digits
    for (uint8_t i = 2; i < kNumDigits; ++i) {
        max7219_cmd(i+1, 0x7F);
    }
}

static void display_timezone()
{
    uint8_t value;

    // Put sign of timezone in first group
    if (_timezoneOffset < 0) {
        max7219_cmd(2, 10 /* - */);
        value = _timezoneOffset * -1;
    } else {
        max7219_cmd(2, 14 /* P */);
        value = _timezoneOffset;
    }

    uint8_t ones = value;
    uint8_t tens = 0;

    while (ones >= 10) {
        ones -= 10;
        ++tens;
    }

    // Put number in the middle
    max7219_cmd(3, tens);
    max7219_cmd(4, ones);

    // Blank other digits
    max7219_cmd(1, 0x7F);
    max7219_cmd(5, 0x7F);
    max7219_cmd(6, 0x7F);
}

static void display_adjust_brightness(const uint8_t reading)
{
    // Map of brightness (index) to minimum ADC reading to trigger
    // Note the 200mV offset reads as around 10 with this configuration
    static const __flash uint8_t brightnessTable[] = {
        30, // 9%  duty cycle
        40, // 15% duty cycle
        50, // 21% duty cycle
        65, // 28% duty cycle
        80, // 34% duty cycle
        95, // 40% duty cycle
        110, // 46% duty cycle
        125, // 53% duty cycle
        140, // 59% duty cycle
        155, // 65% duty cycle
        170, // 71% duty cycle
        185, // 78% duty cycle
        200, // 84% duty cycle
        215, // 90% duty cycle
        230, // 96% duty cycle
    };

    // State to obtain an average of LDR readings
    // The size of this array should  be a power of two
    static uint8_t averageBuffer[16] = {};
    static uint8_t writeIndex = 0;
    static uint16_t runningTotal = 0;

    // Adjust running total with the new value
    runningTotal -= averageBuffer[writeIndex];
    runningTotal += reading;

    // Append new reading
    averageBuffer[writeIndex] = reading;
    writeIndex = (writeIndex + 1) % sizeof(averageBuffer);

    const uint8_t average = runningTotal/sizeof(averageBuffer);

    uint8_t intensity = 0;
    while (brightnessTable[intensity] < average) {
        ++intensity;
    }

    // Set brightness
    max7219_cmd(0x0A, intensity);
}

static void increment_timezone()
{
    ++_timezoneOffset;

    if (_timezoneOffset > 13) {
        _timezoneOffset = -12;
    }
}

int main(void)
{
    setup_pins();
    setup_adc();

    max7219_init();

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
                display_error_code(1);
                break;

            case kGPS_BadFormat:
                // This state is returned if the UART line isn't pulled high (ie. GPS unplugged)
                display_error_code(2);
                break;
        }

        // Handle the combined light sensor and button input
        {
            const uint8_t reading = ADCH;

            // The 200mV offset prevents the LDR output dropping below around 10 in an 8-bit reading
            // When the button is pressed the reading should drop to zero.
            if (reading < 9) {
                do {
                    increment_timezone();
                    display_timezone();
                    _delay_ms(500);
                } while (ADCH < 9);
            } else {
                // Update the display brightness for the ambient light level
                display_adjust_brightness(reading);
            }
        }
    }
}

