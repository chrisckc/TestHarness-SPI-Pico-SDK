// Copyright (c) 2021 Michael Stoops. All rights reserved.
// Portions copyright (c) 2021 Raspberry Pi (Trading) Ltd.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
//    products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Example of an SPI bus slave using the PL022 SPI interface
// Modified by Chris Claxton

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

// Debug Signal outputs
#define LED_BUILTIN (25u)
#define DEBUG_PIN2 (6u)
#define DEBUG_PIN3 (7u)
#define DEBUG_PIN4 (8u)
#define DEBUG_PIN5 (9u)
#define DEBUG_PIN_INITIAL_STATE (1)

//#define SPI_CLOCK (12000 * 1000)
#define SPI_CLOCK (1000 * 1000)
#define SPI_INSTANCE (spi0) // spi_default is spi0
#define SPI_MODE (1) // Cant use SPI Mode 0 without toggling the CS line for every byte otherwise the receiving Pico in Mode 0 does not receive the data.
// Be sure to use the correct pins labeled for SPI0 or SPI1 on the pinout diagram
#define SPI_CS  (17u) // spi0 default:PIN_SPI0_SS   (17)  SS (slave select) signal,
#define SPI_RX  (16u) // spi0 default:PIN_SPI0_MISO (16)  SPI RX pin, MOSI is TX from the SPI Salve device perspective
#define SPI_TX  (19u) // spi0 default:PIN_SPI0_MOSI (19)  SPI TX pin, MISO is RX from the SPI Salve device perspective
#define SPI_SCK (18u) // spi0 default:PIN_SPI0_SCK  (18)  SPI Clock

#define BUF_LEN         0x100 // 256 byte buffer
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];
uint8_t testData = 0xAA; // 0xAA = 170 decimal is binary 10101010 producing a nice pattern on the scope for testing

#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
// Setting this to true breaks it, received data is corrupted
#define CHECK_SPI_STATUS false // Defines if we want to check the status of the SPI bus using spi_is_busy and spi_is_readable

unsigned int seconds = 0, lastSeconds = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, receiveErrorCount = 0, sendCounter = 0, sendErrorCount = 0;
unsigned int receivedBytesErrorCount = 0;


void printbuf(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02X \r\n", buf[i]);
        else
            printf("%02X ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        printf("   \r\n");
    }
}

unsigned int verifyInBuffer(unsigned int page, bool printOnlyFirstError) {
    int errorCount = 0;
    for (int i = 0; i < BUF_LEN; ++i) {
        if (in_buf[i] != i) {
            if (errorCount == 0 && printOnlyFirstError) {
                printf("ERROR! page: %07u First Error at index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i, in_buf[i]);
            } else if (!printOnlyFirstError) {
                printf("ERROR! page: %07u index: %03u expected: 0x%02X received: 0x%02X    \r\n", page, i, i, in_buf[i]);
            }
            errorCount++;
        }
    }
    return errorCount;
}


void clearbuf(uint8_t buf[], size_t len) {
    for (int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}


int main() {
    // Enable UART so we can print
    stdio_init_all();
#if !defined(SPI_INSTANCE) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_slave example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else
    int startupDelay = 5;
    for (int i = 1; i <= startupDelay; ++i) {
        printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    printf("\e[2J\e[H"); // clear screen and go to home position

    printf("SPI slave example using SPI Mode: %d SPI Clock: %d Hz\r\n", SPI_MODE, SPI_CLOCK);
    printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
    printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
    printf("get_core_num: %u \r\n\r\n", get_core_num());

    // Init the onboard LED
    gpio_set_function(LED_BUILTIN, GPIO_FUNC_SIO);
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, GPIO_OUT);
    gpio_put(LED_BUILTIN, 1); // turn on the LED

    // Init the debug pins
    gpio_set_function(DEBUG_PIN2, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN2);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_put(DEBUG_PIN2, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN3, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN3);
    gpio_set_dir(DEBUG_PIN3, GPIO_OUT);
    gpio_put(DEBUG_PIN3, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN4, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN4);
    gpio_set_dir(DEBUG_PIN4, GPIO_OUT);
    gpio_put(DEBUG_PIN4, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN5, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN5);
    gpio_set_dir(DEBUG_PIN5, GPIO_OUT);
    gpio_put(DEBUG_PIN5, DEBUG_PIN_INITIAL_STATE);

    // Setup GPIO's
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CS, GPIO_FUNC_SPI);
    spi_cpol_t cpol; // Clock Polarity
    spi_cpha_t cpha; // Clock Phase
    if (SPI_MODE == 0) {
        cpol = SPI_CPOL_0;
        cpha = SPI_CPHA_0;
    } else if (SPI_MODE == 1) {
        cpol = SPI_CPOL_0;
        cpha = SPI_CPHA_1;
    } else if (SPI_MODE == 2) {
        cpol = SPI_CPOL_1;
        cpha = SPI_CPHA_0;
    } else if (SPI_MODE == 3) {
        cpol = SPI_CPOL_1;
        cpha = SPI_CPHA_1;
    } else {
        cpol = SPI_CPOL_0;
        cpha = SPI_CPHA_0;
    }
    // Enable SPI
    spi_init(SPI_INSTANCE, SPI_CLOCK);
    spi_set_format(SPI_INSTANCE, 8, cpol, cpha, SPI_MSB_FIRST);
    spi_set_slave(SPI_INSTANCE, true);

    // Make the SPI pins available to picotool
    //bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        out_buf[i] = ~i;
    }
    clearbuf(in_buf, BUF_LEN);

    printf("SPI slave says: When reading from MOSI, the value:  0x%02X (%u) and then the following buffer will be written to MISO:\r\n", testData, testData);
    printbuf(out_buf, BUF_LEN);
    printf("\r\n");

    unsigned long startMillis = to_ms_since_boot(get_absolute_time());
    unsigned long currentMillis = 0;
    unsigned int verifyErrorCount = 0;
    bool spiWasRead = false;
    uint8_t receivedData = 0;

    // Loop for ever...
    for (size_t i = 0; ; ++i) {
        gpio_put(LED_BUILTIN, 0); // turn off the LED
        gpio_put(DEBUG_PIN2, 0);
        if (CHECK_SPI_STATUS) printf("loop%u: %ums Checking if SPI bus is busy ... \r\n", i, to_ms_since_boot(get_absolute_time()));
        fflush(stdout); // flush the serial buffer just in case to make sure the above message is outputted
        if (!CHECK_SPI_STATUS || !spi_is_busy(SPI_INSTANCE)) {
            if (CHECK_SPI_STATUS)  printf("loop%u: %ums Checking if SPI bus is readable ... \r\n", i, to_ms_since_boot(get_absolute_time()));
            fflush(stdout);
            if (!CHECK_SPI_STATUS || spi_is_readable(SPI_INSTANCE)) {
                printf("loop%u: %ums Waiting for SPI data ... \r\n", i, to_ms_since_boot(get_absolute_time()));
                fflush(stdout);
                // Read data from the SPI bus and the same time send the testData value to the SPI Master
                spi_write_read_blocking(SPI_INSTANCE, &testData, &receivedData, 1); // The code sits waiting here until the expected number of bytes have been received, in this case 1 byte

                // Write the output buffer to MISO (TX pin), and at the same time read from MOSI (RX Pin).
                spi_write_read_blocking(SPI_INSTANCE, out_buf, in_buf, BUF_LEN); // The code sits waiting here until the expected number of bytes have been received, in this case BUF_LEN
                spiWasRead = true;
                gpio_put(LED_BUILTIN, 1); // turn on the LED
                gpio_put(DEBUG_PIN2, 1);
                printf("loop%u: %ums SPI data received and sent ... \r\n", i, to_ms_since_boot(get_absolute_time()));
            } else {
                printf("loop%u: %ums SPI bus is not readable ... \r\n", i, to_ms_since_boot(get_absolute_time()));
            }
        } else {
            printf("loop%u: %ums SPI bus is busy ... \r\n", i, to_ms_since_boot(get_absolute_time()));
        }
        // Keep track of seconds since start
        currentMillis = to_ms_since_boot(get_absolute_time());
        seconds = (currentMillis - startMillis) / 1000;
        if (seconds - lastSeconds > 0) {
            lastSeconds = seconds;
            //calculate the receive rate, per second
            receiveRate = receiveCounter - lastReceiveCount;
            lastReceiveCount = receiveCounter;
        }
        if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
            printf("\e[H"); // move to the home position, at the upper left of the screen
            printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
        // Print the header info
        printf("\r\nloopCounter(i): %07u         \r\n", i);
        printf("Seconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
        printf("receiveCounter: %07u         \r\n", receiveCounter);
        printf("receiveRate: %07u            \r\n", receiveRate);
        printf("Receive errorCount: %03u         \r\n", receiveErrorCount);
        printf("Receive FailureRate: %11.7f percent  \r\n", 100.0f * receiveErrorCount / (receiveCounter > 0 ? receiveCounter : 1));
        printf("receivedBytesErrorCount: %03u         \r\n", receivedBytesErrorCount);
        printf("Data Received...                                                                \r\n");

        if (spiWasRead) {
            receiveCounter++;
            // Write to stdio whatever came in on the MOSI line.
            printf("SPI slave says: read page %u from the MOSI line, received single byte transfer value: 0x%02X (%03u) \r\n", receiveCounter, receivedData, receivedData);
        } else {
            printf("SPI slave says: ERROR! Reading page %u from the MOSI line, SPI was busy or not readable!\r\n", receiveCounter);
        }

        printf("SPI slave says: contents of receive buffer:\r\n");
        printbuf(in_buf, BUF_LEN);

        if (spiWasRead) {
            printf("SPI Slave says: Verifying received data... \r\n");
            if (receivedData != testData) {
                receiveErrorCount++;
                printf("SPI slave says: ERROR! single byte transfer expected value: 0x%02X (%03u) received value: 0x%02X (%03u) \r\n", testData, testData, receivedData, receivedData);
            } else {
                printf("SPI slave says: OK: single byte transfer expected value: 0x%02X (%03u) received value: 0x%02X (%03u) \r\n", testData, testData, receivedData, receivedData);
            }
            printf("SPI Slave says: Verifying received buffer... \r\n");
            verifyErrorCount = verifyInBuffer(receiveCounter, true);
            receivedBytesErrorCount += verifyErrorCount;
            // Check that we only record the error once for each receive cycle
            if (receivedData == testData && verifyErrorCount > 0) {
                receiveErrorCount++;
            } else {
                printf("SPI Slave says: OK: received buffer verified successfully \r\n");
            }
        } else {
            printf("SPI Slave says: Data not received! \r\n");
            printf("SPI slave says: ERROR! Reading page %u from the MOSI line, SPI was busy or not readable! \r\n", receiveCounter);
            printf("\r\n\r\n");
        }

        clearbuf(in_buf, BUF_LEN);
        spiWasRead = false;
        receivedData = 0;
    }
#endif
}
