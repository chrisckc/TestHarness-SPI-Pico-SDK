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
uint8_t testData = 0xAA; // 0xAA = 170 decimal is binary 10101010 producing a nice pattern on the scope for testing

void printbuf(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02X\n", buf[i]);
        else
            printf("%02X ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        printf("\r\n");
    }
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

    printf("SPI slave example using SPI Mode: %d SPI Clock: %d Hz\r\n\r\n", SPI_MODE, SPI_CLOCK);

    // Enable and connect to GPIOs
    spi_init(SPI_INSTANCE, SPI_CLOCK);
    spi_set_slave(SPI_INSTANCE, true);
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
    spi_set_format(SPI_INSTANCE, 8, cpol, cpha, SPI_MSB_FIRST);

    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        out_buf[i] = ~i;
    }
    clearbuf(in_buf, BUF_LEN);

    printf("SPI slave says: When reading from MOSI, the value:  0x%02X (%u) and then the following buffer will be written to MISO:\r\n", testData, testData);
    printbuf(out_buf, BUF_LEN);
    printf("\r\n");

    // Loop for ever...
    for (size_t i = 0; ; ++i) {

        uint8_t receivedData = 0;
        // Read data from the SPI bus and the same time send the testData value to the SPI Master
        spi_write_read_blocking(SPI_INSTANCE, &testData, &receivedData, 1); // The code sits waiting here until the expected number of bytes have been received, in this case 1 byte

        // Write the output buffer to MISO (TX pin), and at the same time read from MOSI (RX Pin).
        spi_write_read_blocking(SPI_INSTANCE, out_buf, in_buf, BUF_LEN); // The code sits waiting here until the expected number of bytes have been received, in this case BUF_LEN

        // Write to stdio whatever came in on the MOSI line.
        printf("SPI slave says: read page %d from the MOSI line, receivedData value: 0x%02X (%03u) \r\n", i, receivedData, receivedData);
        printbuf(in_buf, BUF_LEN);
        clearbuf(in_buf, BUF_LEN);
        printf("\e[H"); // move to the home position, at the upper left of the screen
        printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    }
#endif
}
