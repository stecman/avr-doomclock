# Tiny GPS Clock

This is a `hh:mm:ss` clock that runs from the time contained in NMEA RMC sentences.
The code is targeted at an Atmel ATTiny13A.

Notes:

- As the target microcontroller doesn't have enough RAM to store a full NMEA sentence,
this project uses a custom parser that only reads RMC ("Recommended Minimum") sentences from a UART
and extracts the time from that (see`nmea.c`).

- The ATTiny13A lacks SPI and UART peripherals, so these are implemented in software.

Schematic coming soon.

## Building and flashing

The `avr-gcc` toolchain is required to build, but there are no other external dependencies.

```sh
# Build
make

# Flash with programmer configured in Makefile
make flash
```

## Testing

A crude test suite for the NMEA decoding can be run with:

```sh
cd test
make
```