# Test Harness for debugging SPI communication issues between 2 Raspberry Pi Pico's (RP2040)

### This is based on the master-slave example from the Pico examples Repo:
https://github.com/raspberrypi/pico-examples/tree/master/spi/spi_master_slave


### I have modified it to improve the serial output and added a separate, single byte, SPI data transfer before the 256 byte buffer is transferred.

I also changed the SPI configuration to use Mode 1 and also spi1 on the master and spi0 on the slave, specifying different pins.
One of reasons for using Mode1 is that the CS line is held low for the duration of multi-byte transfers rather than being toggled after each byte sent as in Mode 0. This looks to be more efficient as there are no gaps in the clock signal.

I also increased the send rate from 1 per second to 10 per second.

### What it does
The Master Pico sends a single value to the Slave Pico, in this case i have chosen 0xAA (170 decimal)
Immediately after this it sends to same 256 byte buffer as used in the original master-slave example.

In response to this, the Slave Pico, as it is receiving the single byte value of 0xAA, sends back the same value 0xAA to the Master Pico as a simultaneous transfer as per the way SPI works. Immediately after this transfer, as in the original example, the Master Pico sends the 256 byte buffer while the Slave Pico simultaneously sends back its own 256 byte buffer in return (a reversed copy of the buffer used by the master)


### The issue
Adding this separate single byte transfer before the 256 byte buffer is sent has caused it to break after it has been running for a short time.

It works fine for the first 100 to 200 transfers and the expected response data is seen by the Master Pico, but then at some random number of transfers later, the Master Pico starts to report incorrect data being sent back from the Slave Pico. instead of reporting 0xAA being sent back it start to report 0x01. The start of the 256 byte buffer response is also corrupted and out of sync as the first byte shows as 0x00, followed by 0xAA and then the expected buffer starting from 0xFF and ending at 0x02 instead of 0x00 due to the apparent shifting of the response.

This error condition continues until the Pico's are reset.

Looking at the Oscilloscope i can see that the Salve Pico stops sending back the correct data when the fault condition appears after the first 100 to 200 transfers.

When the fault appears, it can be seen on the scope that the CS line has stopped going high in between the first single byte transfer and second 256 byte buffer transfer. This could reason why the Slave Pico gets confused about when the 256 byte transfer starts resulting in the data bring sent back to the Master Pico being out of sync. This does not explain why the first single byte transfer is also not being handled correctly by the Slave Pico as by this time, the CS line has gone high after the end of the previous 256 byte transfer. Despite still starting with the CS line high, the first single byte transfer starts to fail and continues to fail until the Pico is restarted.


