#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <iostream>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

using namespace std::chrono_literals;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
auto main() -> int
{
   std::cout << "Orientation Sensor Raw Data Test" << std::endl;

   /* Initialise the sensor */
   if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      std::cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!" << std::endl;
      return 1;
   }

   gpio::sleep(1000ms);

   /* Display the current temperature */
   int8_t temp = bno.getTemp();
   std::cout << "Current Temperature: " << temp << " C" << std::endl;

   bno.setExtCrystalUse(true);

   std::cout << "Calibration status values: 0=uncalibrated, 3=fully calibrated" << std::endl;

   /**************************************************************************/
   /*
       Arduino loop function, called once 'setup' is complete (your own code
       should go here)
   */
   /**************************************************************************/
   while (true) {
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      /* Display the floating point data */
      std::cout << "X: " << euler.x() << " Y: " << euler.y() << " Z: " << euler.z() << std::endl;

      // Quaternion data
      imu::Quaternion quat = bno.getQuat();
      std::cout << "qW: " << quat.w() << " qX: " << quat.x() << " qY: " << quat.y()
                << " qZ: " << quat.z() << std::endl;

      /* Display calibration status for each sensor. */
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      std::cout << "CALIBRATION: Sys =" << std::dec << static_cast<uint16_t>(system)
                << " Gyro = " << std::dec << static_cast<uint16_t>(gyro) << " Accel = " << std::dec
                << static_cast<uint16_t>(accel) << " Mag = " << std::dec
                << static_cast<uint16_t>(mag) << std::endl;

      gpio::sleep(std::chrono::milliseconds(BNO055_SAMPLERATE_DELAY_MS));
   }
}
