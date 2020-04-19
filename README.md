##### Note: This is a port to work on the Odroid platform. 

Adafruit Unified BNO055 Driver (AHRS/Orientation)  
================

<a href="https://www.adafruit.com/product/2472"><img src="assets/board.jpg?raw=true" width="500px"></a>

This driver is for the Adafruit BNO055 Breakout, and is based on Adafruit's Unified Sensor Library (Adafruit_Sensor).

Tested and works great with the Adafruit Si4713 Breakout Board 
To work with the Arduino Zero, the BNO055's ADR pin must be high.
* http://www.adafruit.com/products/2472

## What is the Adafruit Unified Sensor Library? ##

The Adafruit Unified Sensor Library ([Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)) provides a common interface and data type for any supported sensor.  It defines some basic information about the sensor (sensor limits, etc.), and returns standard SI units of a specific type and scale for each supported sensor type.

It provides a simple abstraction layer between your application and the actual sensor HW, allowing you to drop in any comparable sensor with only one or two lines of code to change in your project (essentially the constructor since the functions to read sensor data and get information about the sensor are defined in the base Adafruit_Sensor class).

This is imporant useful for two reasons:

1.) You can use the data right away because it's already converted to SI units that you understand and can compare, rather than meaningless values like 0..1023.

2.) Because SI units are standardised in the sensor library, you can also do quick sanity checks when working with new sensors, or drop in any comparable sensor if you need better sensitivity or if a lower cost unit becomes available, etc. 

Light sensors will always report units in lux, gyroscopes will always report units in rad/s, etc. ... freeing you up to focus on the data, rather than digging through the datasheet to understand what the sensor's raw numbers really mean.

Adafruit invests time and resources providing this open source code.  Please support Adafruit and open-source hardware by purchasing products from Adafruit!

Kevin (KTOWN) Townsend Adafruit Industries.
MIT license, check license.txt for more information
All text above must be included in any redistribution

You need at least GNU version 7 because some of the code uses C++ 17.

To install:
```
# First install dependencies
# Install wiringPi
cd ~ && git clone https://github.com/hardkernel/wiringPi.git
cd wiringPi
./build

# Install Odroid GPIO
cd ~ && git clone https://github.com/ManuelMeraz/OdroidGPIO.git
mkdir OdroidGPIO/build
cd OdroidGPIO/build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON ..
make -j6 
sudo ctest --output-on-failure # Need sudo to acces pins
sudo make install

# Install Adafruit Universal Sensor Library
cd ~ && git clone https://github.com/ManuelMeraz/Odroid_Adafruit_Sensor.git
mkdir Odroid_Adafruit_Sensor/build
cd Odroid_Adafruit_Sensor/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6 
sudo make install

# All dependencies installed, now to install this library
cd ~ && git clone https://github.com/ManuelMeraz/Odroid_Adafruit_BNO055.git
mkdir Odroid_Adafruit_BNO055/build
cd Odroid_Adafruit_BNO055/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6
sudo make install
```

Done! If you want to use the library in your project. In your CMakeLists.txt it's as simple as
```
find_package(OdroidAdafruitBNO055 required)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE OdroidAdafruitBNO055)
```
