# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include/OdroidAdafruitBNO055 CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR lib/cmake/OdroidAdafruitBNO055 CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(directory LIB BIN INCLUDE CMAKE)
    set(var INSTALL_${directory}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

# creates compile_commands.json database for linters
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
    endif()

    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDE's
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # All static libs will be placed here
    link_directories(${PROJECT_BINARY_DIR}/lib)

    # tell find package the version
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
            OdroidAdafruitBNO055ConfigVersion.cmake
            VERSION ${PACKAGE_VERSION}
            COMPATIBILITY AnyNewerVersion
    )
endif()

configure_file(cmake/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

# Set up third party
include(cmake/thirdparty/SetupThirdParty.cmake)
