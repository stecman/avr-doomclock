#include "nmea.h"
#include "softuart.h"

#include <stdbool.h>

enum GPRMCField {
    GPRMC_SentenceType = 0,
    GPRMC_Timestamp, // UTC of position fix
    GPRMC_Validity, // Data status (A=ok, V=navigation receiver warning)
    GPRMC_Latitude, // Latitude of fix
    GPRMC_Latitude_NorthSouth, // N or S
    GPRMC_Longitude, // Longitude of fix
    GPRMC_Longitude_EastWest, // E or W
    GPRMC_SpeedInKnots, // Speed over ground in knots
    GPRMC_TrueCourse, // Track made good in degrees True
    GPRMC_DateStamp, // UT date
    GPRMC_Variation, // Magnetic variation degrees (Easterly var. subtracts from true course)
    GPRMC_Variation_EastWest, // E or W
};

enum NmeaReadState {
    kSearchStart,
    kSkipSentence,
    kReadType,
    kReadFields,
    kChecksumVerify,
};

/**
 * Convert a two character hex string to a byte
 *
 * Note this assumes the input is a two character array, not a null terminated string.
 */
__attribute__((optimize("unroll-loops")))
static inline uint8_t hex2int(char* hexPair)
{
    uint8_t output = 0;

    for (uint8_t i = 0; i < 2; ++i) {

        // Shift result to make room for next nibble
        output <<= 4;

        // Get hex character
        const uint8_t hexChar = hexPair[i];

        // Transform hex character to the 4bit equivalent number
        if (hexChar >= '0' && hexChar <= '9') {
            output |= hexChar - '0';

        } else if (hexChar >= 'A' && hexChar <= 'F') {
            output |= hexChar - 'A' + 10;

        }
    }

    return output;
}

/**
 * Convert a two character numeric string to an 8-bit number
 *
 * Using this saves 50 bytes of program space over the stdlib implementation by
 * knowing that str contains exactly two numeric characters (zero padded if one digit).
 * The AVR stdlib version also uses a 16-bit signed integer, which we don't need.
 */
static inline uint8_t gps_atoi(char *str)
{
    uint8_t result = 0;

    result = (str[0] - '0') * 10; // Tens
    result += str[1] - '0'; // Ones

    return result;
}

GpsReadStatus gps_read_time(GpsTime* output)
{

    uint8_t calculatedChecksum = 0x0;

    // Buffer for storing number pairs read from the GPS
    char buffer[2];
    uint8_t bufIndex = 0;

    // Which field in the output is currently being written to
    uint8_t outputIndex = 0;

    // RMC sentence header (without null termination)
    static const __flash char GPRMC[5] = "GPRMC";
    uint8_t typeStrIndex = 0;

    // Initial states for  sentence matching
    enum NmeaReadState state = kSearchStart;
    enum GPRMCField field = GPRMC_SentenceType;

    // Flag to indicate the decimal portion of time is being skipped
    bool hitTimeDecimal = false;

    // Flag to indicate the date/time field was non-empty
    // During start-up the GPS can return blank fields while it aquires a signal
    bool sawTimeFields = false;

    // NMEA sentences are limited to 79 characters including the start '$' and end '\r\n'
    // Limit iterations to this for sanity
    for (uint8_t i = 79; i != 0; --i) {
        char byte = uart_read_byte();

        switch (state) {
            case kSearchStart: {
                // Bail out if end of line hit
                if (byte == '\n') {
                    return kGPS_NoMatch;
                }

                // Look for start character
                if (byte == '$') {
                    state = kReadType;
                    continue;
                }

                // Not the character we're looking for
                continue;
            }

            case kSkipSentence: {
                // Ignore all further bytes until the sentence ends
                if (byte != '\n') {
                    continue;
                }

                return kGPS_NoMatch;
            }

            case kReadType: {
                // Include sentence type in checksum
                calculatedChecksum ^= byte;

                // Try to match against sentence type we want
                if (byte == GPRMC[typeStrIndex]) {
                    if (typeStrIndex == (sizeof(GPRMC) - 1)) {
                        // Matched last character in the flag we want
                        state = kReadFields;
                    } else {
                        ++typeStrIndex;
                    }
                } else {
                    // Saw a '$' but the sentence type didn't match
                    // Ignore everything further in this message
                    state = kSkipSentence;
                }

                continue;
            }

            case kReadFields: {

                // Asterisk marks the end of the data and start of the checksum
                if (byte == '*') {
                    state = kChecksumVerify;
                    continue;
                }

                // Calculate checksum across sentence contents
                calculatedChecksum ^= byte;

                // Fields are delimited by commas
                if (byte == ',') {
                    ++field;
                    continue;
                }

                switch (field) {
                    case GPRMC_Timestamp: {

                        // Skip the fractional part of the timestamp field as we don't use it
                        // This isn't guaranteed to be present in every message
                        if (hitTimeDecimal || byte == '.') {
                            hitTimeDecimal = true;
                            continue;
                        }

#ifdef ENABLE_GPS_DATE
                        // INTENTIONAL FALL THROUGH TO DATESTAMP
                    }

                    case GPRMC_DateStamp: {
#endif

                        // Collect pairs of characters and convert them to numbers
                        buffer[bufIndex] = byte;
                        bufIndex++;

                        if (bufIndex == 2) {
                            bufIndex = 0;
                            ((uint8_t*) output)[outputIndex] = gps_atoi(buffer);
                            ++outputIndex;
                        } else {
                            continue;
                        }

                        sawTimeFields = true;
                        continue;
                    }

                    default:
                        // Skip other fields
                        continue;
                }
            }

            case kChecksumVerify: {
                uint8_t receivedChecksum = 0x0;

                // Collect checksum
                buffer[bufIndex] = byte;
                bufIndex++;

                if (bufIndex == 2) {
                    receivedChecksum = hex2int(buffer);
                } else {
                    continue;
                }

                if (receivedChecksum == calculatedChecksum) {
                    if (sawTimeFields) {
                        return kGPS_Success;
                    } else {
                        return kGPS_NoSignal;
                    }
                } else {
                    return kGPS_InvalidChecksum;
                }
            }

            default:
                // Entered an unrecognised state: abort
                return kGPS_BadFormat;
        }
    }

    // Something has gone wrong
    // The loop ended, which means the sentence was longer than allowed by NMEA
    return kGPS_BadFormat;
}