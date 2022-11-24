if (TARGET tinyusb_device)
        add_executable(spi_master
                spi_master.c
                )

        # Pull in basic dependencies
        target_link_libraries(spi_master pico_stdlib hardware_spi)

            # enable usb output, disable uart output
        pico_enable_stdio_usb(spi_master 1)
        pico_enable_stdio_uart(spi_master 0)

        # create map/bin/hex file etc.
        pico_add_extra_outputs(spi_master)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
