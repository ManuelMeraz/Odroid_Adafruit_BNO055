list(APPEND examples
        sensorapi
        restore_offsets)

foreach(example IN LISTS examples)
    add_executable(${example} ${example}/${example}.cpp)
    target_link_libraries(${example} PRIVATE OdroidAdafruitBNO055)
endforeach()