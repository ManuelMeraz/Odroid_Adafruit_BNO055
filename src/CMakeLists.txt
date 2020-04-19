add_library(OdroidAdafruitBNO055
        SHARED
        Adafruit_BNO055.cpp)

target_link_libraries(OdroidAdafruitBNO055
        PUBLIC
        OdroidAdafruitSensor
        OdroidGPIO)

target_include_directories(OdroidAdafruitBNO055
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/OdroidAdafruitBNO055>)

install(TARGETS OdroidAdafruitBNO055
        EXPORT OdroidAdafruitBNO055Targets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
        COMPONENT dev)
