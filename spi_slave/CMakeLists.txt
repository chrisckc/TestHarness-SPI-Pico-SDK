if (TARGET tinyusb_device)
        add_executable(spi_slave
                spi_slave.c
                )

        # Pull in basic dependencies
        target_link_libraries(spi_slave pico_stdlib hardware_spi)

        # enable usb output, disable uart output
        pico_enable_stdio_usb(spi_slave 1)
        pico_enable_stdio_uart(spi_slave 0)

        # create map/bin/hex file etc.
        pico_add_extra_outputs(spi_slave)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()

