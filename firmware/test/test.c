#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../nmea.h"

static const char* g_currentSentence = NULL;
static int g_sentenceIdx = 0;

/**
 * Emulated uart for test cases
 */
char uart_read_byte()
{
    char out = g_currentSentence[g_sentenceIdx];

    if (out != '\0') {
        ++g_sentenceIdx;
    }

    return out;
}

// Define tests
typedef struct TestCase {
    const char* description;
    const char* sentence;
    GpsReadStatus expectedStatus;
    GpsTime expectedResult;
} TestCase;

static TestCase testcases[] = {
    {
        .description = "Decode valid RMC sentence 1",
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
        .description = "Decode valid RMC sentence 2",
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
        .description = "Decode valid RMC sentence with an empty time field",
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
        .description = "Decode valid stream of sentences",
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
        .description = "Invalid checksum fails",
        .sentence = "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*14\r\n",
        .expectedStatus = kGPS_InvalidChecksum,
    },

    // Unknown sentences
    {
        .description = "Unknown sentence is ignored (RMB)",
        .sentence = "$GPRMB,A,4.08,L,EGLL,EGLM,5130.02,N,00046.34,W,004.6,213.9,122.9,A*3D\r\n",
        .expectedStatus = kGPS_NoMatch,
    },
    {
        .description = "Unknown sentence is ignored (GSV)",
        .sentence = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n",
        .expectedStatus = kGPS_NoMatch,
    },
    {
        .description = "Unknown sentence is ignored (RMA)",
        .sentence = "$GPRMA,A,llll.ll,N,lllll.ll,W,,,ss.s,ccc,vv.v,W*hh\r\n",
        .expectedStatus = kGPS_NoMatch,
    },

    // Junk values
    {
        .description = "Rejection of an endless bogus message",
        .sentence = "[something very unexpected]", // (endlessly outputs nulls at the end of the string)
        .expectedStatus = kGPS_BadFormat,
    },
    {
        .description = "Unexpected termination of valid looking sentence fails",
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

bool assertPasses(TestCase* test, char** errorMsg)
{
    // Update globals
    g_currentSentence = test->sentence;
    g_sentenceIdx = 0;

    GpsTime output = {0, 0, 0, 0, 0, 0};
    GpsReadStatus status = gps_read_time(&output);

    // Test return value matches expected value
    if (status != test->expectedStatus) {
        asprintf(
            errorMsg,
            "Returned %s when %s expected",
            statusToString[status],
            statusToString[test->expectedStatus]
        );

        return false;
    }

    // Test the output matches the expected date and time
    if (test->expectedStatus == kGPS_Success) {
        bool outputMatches = true;
        for (int i = 0; i < sizeof(GpsTime); i++) {
            if ( ((uint8_t*) &output)[i] != ((uint8_t*) &(test->expectedResult))[i] ) {
                outputMatches = false;
                break;
            }
        }

        if (!outputMatches) {
            asprintf(
                errorMsg,
                "Result '%02d:%02d:%02d %02d/%02d/%02d' did not match test.expectedResult",
                output.hour,
                output.minute,
                output.second,
                output.day,
                output.month,
                output.year
            );

            return false;
        }
    }

    return true;
}

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

int main()
{
    for (int i = 0; i < (sizeof(testcases) / sizeof(testcases[0])); i++) {
        TestCase* test = &testcases[i];

        char* errorMsg = NULL;

        if (assertPasses(test, &errorMsg)) {
            printf(ANSI_COLOR_GREEN " ✓ " ANSI_COLOR_RESET "%s\n", test->description);
        } else {
            printf(ANSI_COLOR_RED " ✗ " ANSI_COLOR_RESET "%s\n\n", test->description);
            printf(
                " %.*s\n\n",
                (int) strcspn(test->sentence, "\r\n"),
                test->sentence
            );
            printf(ANSI_COLOR_RED " FAILED: " ANSI_COLOR_RESET "%s\n\n", errorMsg);

            free(errorMsg);
            return 1;
        }
    }

    return 0;
}