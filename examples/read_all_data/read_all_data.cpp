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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void printEvent(sensors_event_t* event)
{
   double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem
   if (event->type == SENSOR_TYPE_ACCELEROMETER) {
      std::cout << "Accelerometer: ";
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
   } else if (event->type == SENSOR_TYPE_ORIENTATION) {
      std::cout << "Orientation: ";
      x = event->orientation.x;
      y = event->orientation.y;
      z = event->orientation.z;
   } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
      std::cout << "Magnetic Field: ";
      x = event->magnetic.x;
      y = event->magnetic.y;
      z = event->magnetic.z;
   } else if ((event->type == SENSOR_TYPE_GYROSCOPE) ||
              (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
      std::cout << "Gyroscope: ";
      x = event->gyro.x;
      y = event->gyro.y;
      z = event->gyro.z;
   } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
      std::cout << "Acceleration w/o gravity: ";
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
   } else
   {
      std::cout << "Unknown (" << event->type << "): ";
   }

   std::cout << "x = ";
   std::cout << x;
   std::cout << " | y = ";
   std::cout << y;
   std::cout << " | z = ";
   std::cout << z << std::endl;
}

auto main() -> int
{
   /* Initialise the sensor */
   if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      std::cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!" << std::endl;
      return 1;
   }

   gpio::sleep(1000ms);
   while (true) {
      // could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
      sensors_event_t orientationData, angVelocityData, linearAccelData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

      std::cout << "--------------------------------" << std::endl;
      printEvent(&orientationData);
      printEvent(&angVelocityData);
      printEvent(&linearAccelData);

      uint16_t boardTemp = bno.getTemp();
      std::cout << "temperature: " << boardTemp << std::endl;

      gpio::sleep(std::chrono::milliseconds(BNO055_SAMPLERATE_DELAY_MS));
   }
}
