#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <iostream>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
   sensor_t sensor;
   bno.getSensor(&sensor);
   std::cout << "------------------------------------" << std::endl;
   std::cout << "Sensor:       " << sensor.name << std::endl;
   std::cout << "Driver Ver:   " << sensor.version << std::endl;
   std::cout << "Unique ID:    " << sensor.sensor_id << std::endl;
   std::cout << "Max Value:    " << sensor.max_value << " xxx" << std::endl;
   std::cout << "Min Value:    " << sensor.min_value << " xxx" << std::endl;
   std::cout << "Resolution:   " << sensor.resolution << " xxx" << std::endl;
   std::cout << "------------------------------------" << std::endl;
   std::cout << std::endl;
   gpio::sleep(500ms);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
auto main(void) -> int
{
   std::cout << "Orientation Sensor Test" << std::endl;

   /* Initialise the sensor */
   if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      std::cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!" << std::endl;
      return 1;
   }

   gpio::sleep(1000ms);

   /* Use external crystal for better accuracy */
   bno.setExtCrystalUse(true);

   /* Display some basic information on this sensor */
   displaySensorDetails();
   while (true) {
      /* Get a new sensor event */
      sensors_event_t event;
      bno.getEvent(&event);

      /* Board layout:
             +----------+
             |         *| RST   PITCH  ROLL  HEADING
         ADR |*        *| SCL
         INT |*        *| SDA     ^            /->
         PS1 |*        *| GND     |            |
         PS0 |*        *| 3VO     Y    Z-->    \-X
             |         *| VIN
             +----------+
      */

      /* The processing sketch expects data as roll, pitch, heading */
      std::cout << "Orientation: " << static_cast<float>(event.orientation.x) << " "
                << static_cast<float>(event.orientation.y) << " "
                << static_cast<float>(event.orientation.z) << std::endl;

      /* Also send calibration data for each sensor. */
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      std::cout << "CALIBRATION: Sys =" << std::dec << static_cast<uint16_t>(system)
                << " Gyro = " << std::dec << static_cast<uint16_t>(gyro) << " Accel = " << std::dec
                << static_cast<uint16_t>(accel) << " Mag = " << std::dec
                << static_cast<uint16_t>(mag) << std::endl;

      gpio::sleep(std::chrono::milliseconds(BNO055_SAMPLERATE_DELAY_MS));
   }
}
