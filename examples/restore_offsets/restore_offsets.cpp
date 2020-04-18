#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <iomanip>
#include <iostream>

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
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
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
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
   /* Get the system status values (mostly for debugging purposes) */
   uint8_t system_status, self_test_results, system_error;
   system_status = self_test_results = system_error = 0;
   bno.getSystemStatus(&system_status, &self_test_results, &system_error);

   /* Display the results in the Serial Monitor */
   std::cout << std::endl;
   std::cout << "System Status: 0x";
   std::cout << std::hex << system_status << std::endl;
   std::cout << "Self Test:     0x";
   std::cout << std::hex << self_test_results << std::endl;
   std::cout << "System Error:  0x";
   std::cout << std::hex << system_error << std::endl;
   std::cout << std::endl;
   gpio::sleep(500ms);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
   /* Get the four calibration values (0..3) */
   /* Any sensor data reporting 0 should be ignored, */
   /* 3 means 'fully calibrated" */
   uint8_t system, gyro, accel, mag;
   system = gyro = accel = mag = 0;
   bno.getCalibration(&system, &gyro, &accel, &mag);

   /* The data should be ignored until the system calibration is > 0 */
   std::cout << "\t";
   if (!system) {
      std::cout << "! ";
   }

   /* Display the individual values */
   std::cout << "Sys:";
   std::cout << std::dec << system;
   std::cout << " G:";
   std::cout << std::dec << gyro;
   std::cout << " A:";
   std::cout << std::dec << accel;
   std::cout << " M:";
   std::cout << std::dec << mag;
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t& calibData)
{
   std::cout << "Accelerometer: ";
   std::cout << calibData.accel_offset_x << " ";
   std::cout << calibData.accel_offset_y << " ";
   std::cout << calibData.accel_offset_z << " ";

   std::cout << "\nGyro: ";
   std::cout << calibData.gyro_offset_x << " ";
   std::cout << calibData.gyro_offset_y << " ";
   std::cout << calibData.gyro_offset_z << " ";

   std::cout << "\nMag: ";
   std::cout << calibData.mag_offset_x << " ";
   std::cout << calibData.mag_offset_y << " ";
   std::cout << calibData.mag_offset_z << " ";

   std::cout << "\nAccel Radius: ";
   std::cout << calibData.accel_radius;

   std::cout << "\nMag Radius: ";
   std::cout << calibData.mag_radius;
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
auto main() -> int
{
   std::cout << "Orientation Sensor Test" << std::endl;
   std::cout << std::endl;

   /* Initialise the sensor */
   if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      std::cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!" << std::endl;
      return 1;
   }

   adafruit_bno055_offsets_t calibrationData;
   sensor_t sensor;

   /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
   bno.getSensor(&sensor);

   /* Display some basic information on this sensor */
   displaySensorDetails();

   /* Optional: Display current status */
   displaySensorStatus();

   /* Crystal must be configured AFTER loading calibration data into BNO055. */
   bno.setExtCrystalUse(true);

   sensors_event_t event;
   bno.getEvent(&event);
   std::cout << "Please Calibrate Sensor: " << std::endl;
   while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);

      std::cout << "X: ";
      std::cout << std::setprecision(4) << event.orientation.x;
      std::cout << "\tY: ";
      std::cout << std::setprecision(4) << event.orientation.y;
      std::cout << "\tZ: ";
      std::cout << std::setprecision(4) << event.orientation.z;

      /* Optional: Display calibration status */
      displayCalStatus();

      /* New line for the next sample */
      std::cout << std::endl;

      /* Wait the specified delay before requesting new data */
      gpio::sleep(std::chrono::milliseconds(BNO055_SAMPLERATE_DELAY_MS));
   }

   std::cout << "\nFully calibrated!";
   std::cout << "--------------------------------";
   std::cout << "Calibration Results: ";
   adafruit_bno055_offsets_t newCalib;
   bno.getSensorOffsets(newCalib);
   displaySensorOffsets(newCalib);

   std::cout << "\n--------------------------------\n" << std::endl;
   gpio::sleep(500ms);
   while (true) {
      /* Get a new sensor event */
      sensors_event_t event;
      bno.getEvent(&event);

      /* Display the floating point data */
      std::cout << "X: ";
      std::cout << std::setprecision(4) << event.orientation.x;
      std::cout << "\tY: ";
      std::cout << std::setprecision(4) << event.orientation.y;
      std::cout << "\tZ: ";
      std::cout << std::setprecision(4) << event.orientation.z;

      /* Optional: Display calibration status */
      displayCalStatus();

      /* Optional: Display sensor status (debug only) */
      // displaySensorStatus();

      /* New line for the next sample */
      std::cout << std::endl;

      /* Wait the specified delay before requesting new data */
      gpio::sleep(std::chrono::milliseconds(BNO055_SAMPLERATE_DELAY_MS));
   }
}
