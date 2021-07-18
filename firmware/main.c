#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

// Include sources directly so the compiler can optimise everything together
// This saves a significant amount of code space
#include "softuart.c"
#include "nmea.c"

#define PIN_MOSI PB0
#define PIN_SCK PB2
#define PIN_LOAD PB3
#define PIN_LIGHT_SENSE PB4

#define EEPROM_TIMEZONE_ADDR 0
#define kNumDigits 6

static int8_t _timezoneOffset = 0;
static GpsTime _gpsTime = {0, 0, 0};

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

__attribute__ ((unused))
static void eeprom_wait_for_write()
{
    while(EECR & (1<<EEPE));
}

static void unchecked_eeprom_write(uint8_t address, uint8_t data)
{
    // Note: this doesn't wait for completion of any previous write
    // This is a code size optimisation as we only write a single byte infrequently

    // Set Programming mode
    EECR = (0 << EEPM1) | (0 >> EEPM0);

    // Set up address and data registers
    EEARL = address;
    EEDR = data;

    // Write logical one to EEMPE
    EECR |= (1 << EEMPE);
    // Start eeprom write by setting EEPE
    EECR |= (1 << EEPE);
}

static uint8_t unchecked_eeprom_read(uint8_t address)
{
    // Note: this doesn't wait for completion of any previous write
    // This is a code size optimisation as we only read at start-up, before any writes

    // Set up address register
    EEARL = address;

    // Start eeprom read by writing EERE
    EECR |= (1 << EERE);

    // Return data from data register
    return EEDR;
}

/**
 * Clock out a command and data pair to the MAX7219 (SPI-like)
 */
static void spi_send_16(uint16_t value)
{
    // Clock out 8 bits, MSB first
    for (uint8_t i = 16; i != 0; --i) {
        // Bring the clock low
        PORTB &= ~_BV(PIN_SCK);

        // Set output to 0
        PORTB &= ~_BV(PIN_MOSI);

        // Set output to 1 if this bit is set
        if (value & 0x8000) {
            PORTB |= _BV(PIN_MOSI);
        }

        // Bring clock high to send bit
        PORTB |= _BV(PIN_SCK);

        // Next bit
        value <<= 1;
    }
}

/**
 * Write to a register on the MAX7219
 */
static void max7219_cmd(uint8_t address, uint8_t data)
{
    // Select chip (active low)
    PORTB &= ~_BV(PIN_LOAD);

    // Clock out address and data as a combined word for code size savings
    spi_send_16((address << 8) | data);

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
 * Modify the passed time with the current timezone offset
 */
static void apply_timezone_offset(GpsTime* now)
{
    // Adjust hour for timezone
    int8_t hour = now->hour;
    hour += _timezoneOffset;

    if (hour > 23) {
        hour -= 24;
    } else if (hour < 0) {
        hour += 24;
    }

    now->hour = hour;
}

/**
 * Send the current time to the MAX7219 as 6 BCD digits
 */
static void display_update(GpsTime* now)
{
    // Send time to display
    uint8_t digit = 1;

    for (int8_t i = 0; i < 3; ++i) {

        // Manually digit into tens and ones columns
        // This saves 25 bytes vs. using the divide and modulo operators.
        uint8_t ones = ((uint8_t*) now)[i];
        uint8_t tens = 0;

        while (ones >= 10) {
            ones -= 10;
            ++tens;
        }

        max7219_cmd(digit++, tens);
        max7219_cmd(digit++, ones);
    }
}

/**
 * Set all digits on the display to a value with no illuminated segments
 */
static void display_clear()
{
    for (uint8_t i = kNumDigits; i != 0; --i) {
        max7219_cmd(i, 0x7F);
    }
}

static void display_no_signal()
{
    static uint8_t waitIndicator = 0;

    display_clear();

    // Turn on the decimal point on one digit (digits are 1-indexed)
    max7219_cmd(waitIndicator + 1, 0x8F);

    ++waitIndicator;
    if (waitIndicator == kNumDigits) {
        waitIndicator = 0;
    }
}

static void display_error_code(uint8_t code)
{
    display_clear();

    // Display error code
    max7219_cmd(1, 11 /* E */);
    max7219_cmd(2, code);
}

static void display_timezone()
{
    display_clear();

    // Put sign of timezone in the hours column
    uint8_t prefix;
    uint8_t value;

    if (_timezoneOffset < 0) {
        prefix = 10; /* - */
        value = _timezoneOffset * -1;
    } else {
        prefix = 14; /* P */
        value = _timezoneOffset;
    }

    max7219_cmd(2, prefix);

    // Split value into tens and ones columns manually to save code size
    uint8_t ones = value;
    uint8_t tens = 0;

    while (ones >= 10) {
        ones -= 10;
        ++tens;
    }

    // Put number in the middle
    max7219_cmd(3, tens);
    max7219_cmd(4, ones);
}

static void increment_timezone()
{
    ++_timezoneOffset;

    if (_timezoneOffset > 13) {
        _timezoneOffset = -12;
    }
}

static void restore_timezone()
{
    const int8_t timezone = unchecked_eeprom_read(EEPROM_TIMEZONE_ADDR);

    // Restore if the value read from eeprom looks like a timezone
    if (timezone >= -12 && timezone <= 13) {
        _timezoneOffset = timezone;
    }
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
    while (intensity < sizeof(brightnessTable) && brightnessTable[intensity] < average) {
        ++intensity;
    }

    // Set brightness
    max7219_cmd(0x0A, intensity);
}

int main(void)
{
    setup_pins();
    setup_adc();

    max7219_init();

    restore_timezone();

    while (true) {
        // Wait for a line of text from the GPS unit
        const GpsReadStatus status = gps_read_time(&_gpsTime);

        switch (status) {
            case kGPS_Success:
                // Update the display with the new parsed time
                apply_timezone_offset(&_gpsTime);
                display_update(&_gpsTime);
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
            uint8_t numReads = 0;

            // The 200mV offset prevents the LDR output dropping below around 10 in an 8-bit reading
            // When the button is pressed the reading should drop to zero.
            const uint8_t buttonThreshold = 8;

            if (reading < buttonThreshold) {
                const int8_t oldTimezone = _timezoneOffset;

                while (ADCH < buttonThreshold) {
                    ++numReads;

                    // Require the reading to stay below the threshold for a series of readings
                    // (During real world use the ADC reading was occasionally dipping below the
                    // threshold without a button press, causing the timezone to increment unexpectedly.
                    if (numReads <= 20) {
                        _delay_ms(25);
                        continue;
                    }

                    // Reset period counter
                    numReads = 0;

                    // Update timezone
                    increment_timezone();
                    display_timezone();

                }

                // Persist the timezone if it was changed
                if (oldTimezone != _timezoneOffset) {
                    unchecked_eeprom_write(EEPROM_TIMEZONE_ADDR, _timezoneOffset);
                }

            } else {
                // Update the display brightness for the ambient light level
                display_adjust_brightness(reading);
            }
        }
    }
}

