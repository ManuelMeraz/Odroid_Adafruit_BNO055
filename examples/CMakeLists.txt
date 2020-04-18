list(APPEND examples
        serial_loopback
        led_on_off
        pwm_duty_cycle
        I2C_example)

foreach(example IN LISTS examples)
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PRIVATE OdroidGPIO)
endforeach()