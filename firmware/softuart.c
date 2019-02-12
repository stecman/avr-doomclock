#include "softuart.h"

#include <avr/io.h>
#include <util/delay.h>

// Number of microseconds for a 0.5 bit delay at the desired baud rate
// Calculated as ((1/BPS)/0.000001)/2
#define UART_DELAY_US 49 // ~9600 baud

/**
 * Read a single byte transmitted by the GPS
 */
uint8_t uart_read_byte()
{
    uint8_t data = 0x0;

    // Wait for line to go low (start bit)
    while ((PINB & _BV(PIN_SOFT_RX)) != 0);

    // 0.5 bit delay
    _delay_us(UART_DELAY_US);

    // Shift 8 data bits + 1 stop bit at the baud-rate determined by UART_DELAY_MS
    uint8_t bit = 9;
    do {
        --bit;

        // 1 bit delay
        _delay_us(UART_DELAY_US);
        _delay_us(UART_DELAY_US);

        // If this is the stop-bit, don't try to store the value
        if (bit == 0) {
            break;
        }

        // Shift to make room for next bit (reading most-significant bit first)
        data >>= 1;

        if ((PINB & _BV(PIN_SOFT_RX)) != 0) {
            data |= 0x80;
        }
    } while (bit != 0);

    return data;
}