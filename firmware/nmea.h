#pragma once

#include <stdint.h>

typedef struct GpsTime {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
#ifdef ENABLE_GPS_DATE
    uint8_t day;
    uint8_t month;
    uint8_t year;
#endif
} __attribute__((packed)) GpsTime;

typedef enum GpsReadStatus {
    // GPS date and time was successfully read into output parameter
    kGPS_Success = 0,

    // RMC sentence found, but it had no date/time information
    kGPS_NoSignal,

    // Partial sentence or unknown sentence type (this only supports RMC sentences)
    kGPS_NoMatch,

    // Time was read into output parameter, but the calculated checksum failed to match
    kGPS_InvalidChecksum,

    // The sentence had too many characters or fields and could not be parsed
    kGPS_BadFormat,
} GpsReadStatus;

/**
 * Attempt to match GPRMC sentence in the output of uart_read_byte()
 *
 * The output parameter may be altered regardless of success/failure. In the case a non-success
 * status is returned, the struct should be considered in an invalid state
 */
GpsReadStatus gps_read_time(GpsTime* output);