#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../nmea.h"

static const char* currentSentence = NULL;
static int sentenceIdx = 0;

/**
 * Emulated uart for test cases
 */
char uart_read_byte()
{
    char out = currentSentence[sentenceIdx];

    if (out != '\0') {
        ++sentenceIdx;
    }

    return out;
}

// Define tests
typedef struct TestCase {
    const char* sentence;
    GpsReadStatus expectedStatus;
    DateTime expectedResult;
} TestCase;

static TestCase testcases[] = {
    {
        .sentence = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\r\n",
        .expectedStatus = kGPS_Success,
        .expectedResult = {
            .hour = 8,
            .minute = 18,
            .second = 36,
            .day = 13,
            .month = 9,
            .year = 98
        },
    },
    {
        .sentence = "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r\n",
        .expectedStatus = kGPS_Success,
        .expectedResult = {
            .hour = 22,
            .minute = 5,
            .second = 16,
            .day = 13,
            .month = 6,
            .year = 94
        },
    },
    {
        .sentence = "$GPRMC,091502.00,V,,,,,,,040219,,,N*7C\r\n",
        .expectedStatus = kGPS_Success,
        .expectedResult = {
            .hour = 9,
            .minute = 15,
            .second = 2,
            .day = 4,
            .month = 2,
            .year = 19
        },
    },
    {
        // Check sequential strings are parsed individually
        .sentence = "$GPRMC,105445.00,V,,,,,,,040219,,,N*72\r\n$GPVTG,,,,,,,,,N*30\r\n$GPGGA,105445.00,,,,,0,00,99.99,,,,,,*67\r\n",
        .expectedStatus = kGPS_Success,
        .expectedResult = {
            .hour = 10,
            .minute = 54,
            .second = 45,
            .day = 4,
            .month = 2,
            .year = 19
        },
    },
    {
        .sentence = "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*14\r\n",
        .expectedStatus = kGPS_InvalidChecksum,
    },

    // Unknown sentences
    {
        .sentence = "$GPRMB,A,4.08,L,EGLL,EGLM,5130.02,N,00046.34,W,004.6,213.9,122.9,A*3D\r\n",
        .expectedStatus = kGPS_NoMatch,
    },
    {
        .sentence = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n",
        .expectedStatus = kGPS_NoMatch,
    },
    {
        .sentence = "$GPRMA,A,llll.ll,N,lllll.ll,W,,,ss.s,ccc,vv.v,W*hh\r\n",
        .expectedStatus = kGPS_NoMatch,
    },

    // Junk values
    {
        // Test rejection of an endless bogus message (filled with nulls by gps_read_byte())
        .sentence = "[something very unexpected]",
        .expectedStatus = kGPS_BadFormat,
    },
    {
        // Test something that looks like the right sentence is rejected when not formatted as expected
        .sentence = "$GPRMC,but,not,really\r\n",
        .expectedStatus = kGPS_BadFormat,
    },
};

/**
 * Map status numbers to names
 */
static char* statusToString[] = {
    "kGPS_Success",
    "kGPS_NoSignal",
    "kGPS_NoMatch",
    "kGPS_InvalidChecksum",
    "kGPS_BadFormat",
};

int main()
{
    bool didFail = false;

    for (int i = 0; i < (sizeof(testcases) / sizeof(testcases[0])); i++) {
        TestCase test = testcases[i];

        printf("\nTesting sentence:\n    %s", test.sentence);

        currentSentence = test.sentence;
        sentenceIdx = 0;

        DateTime output = {0, 0, 0, 0, 0, 0};
        GpsReadStatus status = gps_read_time(&output);

        // Test return value matches expected value
        if (status != test.expectedStatus) {
            printf(
                "        --> Test failed: Returned %s when %s expected\n",
                statusToString[status],
                statusToString[test.expectedStatus]
            );
            didFail = true;
            continue;
        }

        // Test the output matches the expected date and time
        if (test.expectedStatus == kGPS_Success) {
            bool outputMatches = true;
            for (int i = 0; i < sizeof(DateTime); i++) {
                if ( ((uint8_t*) &output)[i] != ((uint8_t*) &(test.expectedResult))[i] ) {
                    outputMatches = false;
                    break;
                }
            }

            if (!outputMatches) {
                printf("        --> Test failed: result (");

                for (int i = 0; i < sizeof(DateTime); i++) {
                    printf("%d ", ((uint8_t*) &output)[i]);
                }

                printf(") did not match expected value.\n");

                didFail = true;
            }
        }
    }

    return didFail;
}