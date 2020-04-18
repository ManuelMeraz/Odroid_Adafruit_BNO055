cmake_minimum_required(VERSION 3.10)
project(OdroidAdafruitBNO055
        VERSION 0.0.1
        DESCRIPTION "Odroid C++ version of Adafruit BNO055"
        LANGUAGES CXX C)

include(cmake/Setup.cmake)

add_subdirectory(src)

# Add all targets to the build-tree export set
export(TARGETS OdroidAdafruitBNO055 FILE "${PROJECT_BINARY_DIR}/OdroidAdafruitBNO055Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE OdroidAdafruitBNO055)

# Create the OdroidAdafruitBNO055Config.cmake and OdroidAdafruitBNO055ConfigVersion.cmake files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(cmake/OdroidAdafruitBNO055Config.cmake.in "${PROJECT_BINARY_DIR}/OdroidAdafruitBNO055Config.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${ODROIDADAFRUITBNO055_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/OdroidAdafruitBNO055Config.cmake.in "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/OdroidAdafruitBNO055Config.cmake" @ONLY)
# ... for both
configure_file(cmake/OdroidAdafruitBNO055ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/OdroidAdafruitBNO055ConfigVersion.cmake" @ONLY)

# Install the OdroidAdafruitBNO055Config.cmake and OdroidAdafruitBNO055ConfigVersion.cmake
install(FILES
        "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/OdroidAdafruitBNO055Config.cmake"
        "${PROJECT_BINARY_DIR}/OdroidAdafruitBNO055ConfigVersion.cmake"
        DESTINATION
        "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR})

# Install the export set for use with the install-tree
install(EXPORT OdroidAdafruitBNO055Targets
        DESTINATION "${INSTALL_CMAKE_DIR}"
        COMPONENT dev)
