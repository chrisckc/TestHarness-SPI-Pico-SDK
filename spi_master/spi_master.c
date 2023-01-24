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
// Example of an SPI bus master using the PL022 SPI interface
// Modified by Chris Claxton

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

//#define SPI_CLOCK (12000 * 1000)
#define SPI_CLOCK (1000 * 1000)
#define SPI_INSTANCE (spi1) // spi_default is spi0
#define SPI_MODE (1) // Cant use SPI Mode 0 without toggling the CS line for every byte otherwise the receiving Pico in Mode 0 does not receive the data.
// Be sure to use the correct pins labeled for SPI0 or SPI1 on the pinout diagram
#define SPI_CS  (13u) // spi1 default:PIN_SPI1_SS   (13u)  SS (slave select) signal,
#define SPI_TX  (15u) // spi1 default:PIN_SPI1_MOSI (15u)  SPI TX pin, MOSI is TX from the SPI Master device perspective
#define SPI_RX  (12u) // spi1 default:PIN_SPI1_MISO (12u)  SPI RX pin, MISO is RX from the SPI Master device perspective
#define SPI_SCK (14u) // spi1 default:PIN_SPI1_SCK  (14u)  SPI Clock

#define BUF_LEN         0x100 // 256 byte buffer
uint8_t testData = 0xAA; // 0xAA = 170 decimal is binary 10101010 producing a nice pattern on the scope for testing
uint32_t sendInterval = 100; // send every 100 milliseconds


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

void clearbuf(uint8_t buf[], size_t len) {
    for (int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}

int main() {
    // Enable UART so we can print
    stdio_init_all();
#if !defined(SPI_INSTANCE) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_master example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else
    int startupDelay = 6;
    for (int i = 1; i <= startupDelay; ++i) {
        printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    printf("\e[2J\e[H"); // clear screen and go to home position

    printf("SPI master example using SPI Mode: %d SPI Clock: %d Hz\r\n", SPI_MODE, SPI_CLOCK);
    printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
    printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
    printf("get_core_num: %u \r\n\r\n\r\n", get_core_num());

    // Enable and connect to GPIOs
    spi_init(SPI_INSTANCE, SPI_CLOCK);
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
        out_buf[i] = i;
    }
    clearbuf(in_buf, BUF_LEN);

    printf("SPI master says: The value 0x%02X (%u) followed immediately by the buffer printed below will be written to MOSI endlessly every %u mS:\r\n", testData, testData, sendInterval);
    printbuf(out_buf, BUF_LEN);
    printf("\r\n");
    printf("The value 0x%02X (%u) is expected to be returned followed by a reversed version of the above buffer\r\n\r\n", testData, testData);


    for (size_t i = 0; ; ++i) {
        // First send the data length value so the receiver knows what to expect next
        uint8_t returnValue = 0;
        //Send data to the SPI bus and the same time read any data sent from the SPI Slave device into returnValue
        spi_write_read_blocking(SPI_INSTANCE, &testData, &returnValue, 1); // The code sits waiting here until the expected number of bytes have been received, in this case 1 byte

        // Write the output buffer to MOSI (TX Pin), and at the same time read from MISO (RX Pin).
        spi_write_read_blocking(SPI_INSTANCE, out_buf, in_buf, BUF_LEN);  // The code sits waiting here until the expected number of bytes have been received, in this case BUF_LEN

        // Write to stdio whatever came in on the MISO line.
        printf("SPI master says: read page %u from the MISO (RX Pin) line, returnValue value: 0x%02X (%03u) \r\n", i, returnValue, returnValue);
        printbuf(in_buf, BUF_LEN);
        clearbuf(in_buf, BUF_LEN);
        printf("\e[H"); // move to the home position, at the upper left of the screen
        printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

        // Sleep for sendInterval
        sleep_ms(sendInterval);
    }
#endif
}
