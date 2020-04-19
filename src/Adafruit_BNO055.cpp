/*!
 * @file Adafruit_BNO055.cpp
 *
 *  @mainpage Adafruit BNO055 Orientation Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_BNO055.h"

#include <cmath>

using namespace std::chrono_literals;

/*!
 *  @brief  Instantiates a new Adafruit_BNO055 class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 */
Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address) :
   m_sensorID{sensorID},
   m_address{address},
   m_mode{OPERATION_MODE_NDOF},
   m_bus{gpio::get<gpio::I2C::Bus>(address)}
{}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode)
{
   /* Make sure we have the right device */
   uint8_t id = m_bus.read_8_bits(BNO055_CHIP_ID_ADDR);
   if (id != BNO055_ID) {
      gpio::sleep(1000ms); // hold on for boot
      id = m_bus.read_8_bits(BNO055_CHIP_ID_ADDR);
      if (id != BNO055_ID) {
         return false; // still not? ok bail
      }
   }

   /* Switch to config mode (just in case since this is the default) */
   setMode(OPERATION_MODE_CONFIG);

   /* Reset */
   m_bus.write_8_bits(BNO055_SYS_TRIGGER_ADDR, 0x20);
   /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
   gpio::sleep(30ms);
   while (m_bus.read_8_bits(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
      gpio::sleep(10ms);
   }
   gpio::sleep(50ms);

   /* Set to normal power mode */
   m_bus.write_8_bits(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
   gpio::sleep(10ms);

   m_bus.write_8_bits(BNO055_PAGE_ID_ADDR, 0);

   /* Set the output units */
   /*
   uint8_t unitsel = (0 << 7) | // Orientation = Android
                     (0 << 4) | // Temperature = Celsius
                     (0 << 2) | // Euler = Degrees
                     (1 << 1) | // Gyro = Rads
                     (0 << 0);  // Accelerometer = m/s^2
   m_bus.write_8_bits(BNO055_UNIT_SEL_ADDR, unitsel);
   */

   /* Configure axis mapping (see section 3.4) */
   /*
   m_bus.write_8_bits(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
   gpio::sleep(10ms);
   m_bus.write_8_bits(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
   gpio::sleep(10ms);
   */

   m_bus.write_8_bits(BNO055_SYS_TRIGGER_ADDR, 0x0);
   gpio::sleep(10ms);
   /* Set the requested operating mode (see section 3.3) */
   setMode(mode);
   gpio::sleep(20ms);

   return true;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode)
{
   m_mode = mode;
   m_bus.write_8_bits(BNO055_OPR_MODE_ADDR, m_mode);
   gpio::sleep(30ms);
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [REMAP_CONFIG_P0
 *           REMAP_CONFIG_P1 (default)
 *           REMAP_CONFIG_P2
 *           REMAP_CONFIG_P3
 *           REMAP_CONFIG_P4
 *           REMAP_CONFIG_P5
 *           REMAP_CONFIG_P6
 *           REMAP_CONFIG_P7]
 */
void Adafruit_BNO055::setAxisRemap(adafruit_bno055_axis_remap_config_t remapcode)
{
   adafruit_bno055_opmode_t modeback = m_mode;

   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);
   m_bus.write_8_bits(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
   gpio::sleep(10ms);
   /* Set the requested operating mode (see section 3.3) */
   setMode(modeback);
   gpio::sleep(20ms);
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [REMAP_SIGN_P0
 *           REMAP_SIGN_P1 (default)
 *           REMAP_SIGN_P2
 *           REMAP_SIGN_P3
 *           REMAP_SIGN_P4
 *           REMAP_SIGN_P5
 *           REMAP_SIGN_P6
 *           REMAP_SIGN_P7]
 */
void Adafruit_BNO055::setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign)
{
   adafruit_bno055_opmode_t modeback = m_mode;

   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);
   m_bus.write_8_bits(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
   gpio::sleep(10ms);
   /* Set the requested operating mode (see section 3.3) */
   setMode(modeback);
   gpio::sleep(20ms);
}

[[maybe_unused]] /*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
void Adafruit_BNO055::setExtCrystalUse(bool usextal)
{
   adafruit_bno055_opmode_t modeback = m_mode;

   /* Switch to config mode (just in case since this is the default) */
   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);
   m_bus.write_8_bits(BNO055_PAGE_ID_ADDR, 0);
   if (usextal) {
      m_bus.write_8_bits(BNO055_SYS_TRIGGER_ADDR, 0x80);
   } else {
      m_bus.write_8_bits(BNO055_SYS_TRIGGER_ADDR, 0x00);
   }
   gpio::sleep(10ms);
   /* Set the requested operating mode (see section 3.3) */
   setMode(modeback);
   gpio::sleep(20ms);
}

[[maybe_unused]] /*!
 *   @brief  Gets the latest system status info
 *   @param  system_status
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
void Adafruit_BNO055::getSystemStatus(uint8_t* system_status,
                                      uint8_t* self_test_result,
                                      uint8_t* system_error)
{
   m_bus.write_8_bits(BNO055_PAGE_ID_ADDR, 0);

   /* System Status (see section 4.3.58)
      0 = Idle
      1 = System Error
      2 = Initializing Peripherals
      3 = System Iniitalization
      4 = Executing Self-Test
      5 = Sensor fusio algorithm running
      6 = System running without fusion algorithms
    */

   if (system_status != nullptr) {
      *system_status = m_bus.read_8_bits(BNO055_SYS_STAT_ADDR);
   }

   /* Self Test Results
      1 = test passed, 0 = test failed

      Bit 0 = Accelerometer self test
      Bit 1 = Magnetometer self test
      Bit 2 = Gyroscope self test
      Bit 3 = MCU self test

      0x0F = all good!
    */

   if (self_test_result != nullptr) {
      *self_test_result = m_bus.read_8_bits(BNO055_SELFTEST_RESULT_ADDR);
   }

   /* System Error (see section 4.3.59)
      0 = No error
      1 = Peripheral initialization error
      2 = System initialization error
      3 = Self test result failed
      4 = Register map value out of range
      5 = Register map address out of range
      6 = Register map write error
      7 = BNO low power mode not available for selected operat ion mode
      8 = Accelerometer power mode not available
      9 = Fusion algorithm configuration error
      A = Sensor configuration error
    */

   if (system_error != nullptr) {
      *system_error = m_bus.read_8_bits(BNO055_SYS_ERR_ADDR);
   }

   gpio::sleep(200ms);
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
void Adafruit_BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
{
   uint8_t a;
   uint8_t b;

   memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

   /* Check the accelerometer revision */
   info->accel_rev = m_bus.read_8_bits(BNO055_ACCEL_REV_ID_ADDR);

   /* Check the magnetometer revision */
   info->mag_rev = m_bus.read_8_bits(BNO055_MAG_REV_ID_ADDR);

   /* Check the gyroscope revision */
   info->gyro_rev = m_bus.read_8_bits(BNO055_GYRO_REV_ID_ADDR);

   /* Check the SW revision */
   info->bl_rev = m_bus.read_8_bits(BNO055_BL_REV_ID_ADDR);

   a = m_bus.read_8_bits(BNO055_SW_REV_ID_LSB_ADDR);
   b = m_bus.read_8_bits(BNO055_SW_REV_ID_MSB_ADDR);
   info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void Adafruit_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
   uint8_t calData = m_bus.read_8_bits(BNO055_CALIB_STAT_ADDR);
   if (sys != NULL) {
      *sys = (calData >> 6) & 0x03;
   }
   if (gyro != NULL) {
      *gyro = (calData >> 4) & 0x03;
   }
   if (accel != NULL) {
      *accel = (calData >> 2) & 0x03;
   }
   if (mag != NULL) {
      *mag = calData & 0x03;
   }
}

[[maybe_unused]] /*!
                  *  @brief  Gets the temperature in degrees celsius
                  *  @return temperature in degrees celsius
                  */
int8_t
Adafruit_BNO055::getTemp()
{
   int8_t temp = (int8_t)(m_bus.read_8_bits(BNO055_TEMP_ADDR));
   return temp;
}

/*!
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 */
imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t vector_type)
{
   imu::Vector<3> xyz;

   int16_t x, y, z;
   x = y = z = 0;

   constexpr auto BUFFER_SIZE = 6;
   const auto buffer = m_bus.read<BUFFER_SIZE>(static_cast<adafruit_bno055_reg_t>(vector_type));

   x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
   y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
   z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

   /*!
    * Convert the value to an appropriate range (section 3.6.4)
    * and assign the value to the Vector type
    */
   switch (vector_type) {
   case VECTOR_MAGNETOMETER:
      /* 1uT = 16 LSB */
      xyz[0] = ((double)x) / 16.0;
      xyz[1] = ((double)y) / 16.0;
      xyz[2] = ((double)z) / 16.0;
      break;
   case VECTOR_GYROSCOPE:
      /* 1dps = 16 LSB */
      xyz[0] = ((double)x) / 16.0;
      xyz[1] = ((double)y) / 16.0;
      xyz[2] = ((double)z) / 16.0;
      break;
   case VECTOR_EULER:
      /* 1 degree = 16 LSB */
      xyz[0] = ((double)x) / 16.0;
      xyz[1] = ((double)y) / 16.0;
      xyz[2] = ((double)z) / 16.0;
      break;
   case VECTOR_ACCELEROMETER:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x) / 100.0;
      xyz[1] = ((double)y) / 100.0;
      xyz[2] = ((double)z) / 100.0;
      break;
   case VECTOR_LINEARACCEL:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x) / 100.0;
      xyz[1] = ((double)y) / 100.0;
      xyz[2] = ((double)z) / 100.0;
      break;
   case VECTOR_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x) / 100.0;
      xyz[1] = ((double)y) / 100.0;
      xyz[2] = ((double)z) / 100.0;
      break;
   }

   return xyz;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
imu::Quaternion Adafruit_BNO055::getQuat()
{
   int16_t x, y, z, w;
   x = y = z = w = 0;

   /* Read quat data (8 bytes) */
   constexpr auto BUFFER_SIZE = 8;
   const auto buffer = m_bus.read<BUFFER_SIZE>(BNO055_QUATERNION_DATA_W_LSB_ADDR);
   w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
   x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
   y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
   z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

   /*!
    * Assign to Quaternion
    * See
    * http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
    * 3.6.5.5 Orientation (Quaternion)
    */
   const double scale = (1.0 / (1 << 14));
   imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
   return quat;
}

/*!
 *  @brief  Provides the sensor_t data for this sensor
 *  @param  sensor
 *          Sensor description
 */
void Adafruit_BNO055::getSensor(sensor_t* sensor)
{
   /* Clear the sensor_t object */
   memset(sensor, 0, sizeof(sensor_t));

   /* Insert the sensor name in the fixed length char array */
   strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
   sensor->name[sizeof(sensor->name) - 1] = 0;
   sensor->version = 1;
   sensor->sensor_id = m_sensorID;
   sensor->type = SENSOR_TYPE_ORIENTATION;
   sensor->min_value = 0.0F;
   sensor->max_value = 0.0F;
   sensor->min_value = 0.0F;
   sensor->resolution = 0.01F;
}

/*!
 *  @brief  Reads the sensor and returns the data as a sensors_event_t
 *  @param  event
 *          Event description
 *  @return always returns true
 */
bool Adafruit_BNO055::getEvent(sensors_event_t* event)
{
   /* Clear the event */
   memset(event, 0, sizeof(sensors_event_t));

   event->version = sizeof(sensors_event_t);
   event->sensor_id = m_sensorID;
   event->type = SENSOR_TYPE_ORIENTATION;
   event->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::high_resolution_clock::now().time_since_epoch())
                         .count();

   /* Get a Euler angle sample for orientation */
   imu::Vector<3> euler = getVector(Adafruit_BNO055::VECTOR_EULER);
   event->orientation.x = euler.x();
   event->orientation.y = euler.y();
   event->orientation.z = euler.z();

   return true;
}

[[maybe_unused]] /*!
 *  @brief  Reads the sensor and returns the data as a sensors_event_t
 *  @param  event
 *          Event description
 *  @param  vec_type
 *          specify the type of reading
 *  @return always returns true
 */
bool Adafruit_BNO055::getEvent(sensors_event_t* event, adafruit_vector_type_t vec_type)
{
   /* Clear the event */
   memset(event, 0, sizeof(sensors_event_t));

   event->version = sizeof(sensors_event_t);
   event->sensor_id = m_sensorID;
   event->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::high_resolution_clock::now().time_since_epoch())
                         .count();

   // read the data according to vec_type
   imu::Vector<3> vec;
   if (vec_type == Adafruit_BNO055::VECTOR_LINEARACCEL) {
      event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
      vec = getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      event->acceleration.x = vec.x();
      event->acceleration.y = vec.y();
      event->acceleration.z = vec.z();
   } else if (vec_type == Adafruit_BNO055::VECTOR_ACCELEROMETER) {
      event->type = SENSOR_TYPE_ACCELEROMETER;
      vec = getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

      event->acceleration.x = vec.x();
      event->acceleration.y = vec.y();
      event->acceleration.z = vec.z();
   } else if (vec_type == Adafruit_BNO055::VECTOR_GRAVITY) {
      event->type = SENSOR_TYPE_ACCELEROMETER;
      vec = getVector(Adafruit_BNO055::VECTOR_GRAVITY);

      event->acceleration.x = vec.x();
      event->acceleration.y = vec.y();
      event->acceleration.z = vec.z();
   } else if (vec_type == Adafruit_BNO055::VECTOR_EULER) {
      event->type = SENSOR_TYPE_ORIENTATION;
      vec = getVector(Adafruit_BNO055::VECTOR_EULER);

      event->orientation.x = vec.x();
      event->orientation.y = vec.y();
      event->orientation.z = vec.z();
   } else if (vec_type == Adafruit_BNO055::VECTOR_GYROSCOPE) {
      event->type = SENSOR_TYPE_ROTATION_VECTOR;
      vec = getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      event->gyro.x = vec.x();
      event->gyro.y = vec.y();
      event->gyro.z = vec.z();
   } else if (vec_type == Adafruit_BNO055::VECTOR_MAGNETOMETER) {
      event->type = SENSOR_TYPE_MAGNETIC_FIELD;
      vec = getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

      event->magnetic.x = vec.x();
      event->magnetic.y = vec.y();
      event->magnetic.z = vec.z();
   }

   return true;
}

[[maybe_unused]] /*!
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset (buffer size should be 22)
 *  @return true if read is successful
 */
bool Adafruit_BNO055::getSensorOffsets(uint8_t* calibData)
{
   if (isFullyCalibrated()) {
      adafruit_bno055_opmode_t lastMode = m_mode;
      setMode(OPERATION_MODE_CONFIG);

      constexpr size_t BUFFER_SIZE = NUM_BNO055_OFFSET_REGISTERS;
      auto data = m_bus.read<BUFFER_SIZE>(ACCEL_OFFSET_X_LSB_ADDR);
      *calibData = *reinterpret_cast<uint8_t*>(&data[0]);

      setMode(lastMode);
      return true;
   }
   return false;
}

[[maybe_unused]] /*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool Adafruit_BNO055::getSensorOffsets(adafruit_bno055_offsets_t& offsets_type)
{
   if (isFullyCalibrated()) {
      adafruit_bno055_opmode_t lastMode = m_mode;
      setMode(OPERATION_MODE_CONFIG);
      gpio::sleep(25ms);

      /* Accel offset range depends on the G-range:
         +/-2g  = +/- 2000 mg
         +/-4g  = +/- 4000 mg
         +/-8g  = +/- 8000 mg
         +/-1§g = +/- 16000 mg */
      offsets_type.accel_offset_x = (m_bus.read_8_bits(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                    (m_bus.read_8_bits(ACCEL_OFFSET_X_LSB_ADDR));
      offsets_type.accel_offset_y = (m_bus.read_8_bits(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                    (m_bus.read_8_bits(ACCEL_OFFSET_Y_LSB_ADDR));
      offsets_type.accel_offset_z = (m_bus.read_8_bits(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                    (m_bus.read_8_bits(ACCEL_OFFSET_Z_LSB_ADDR));

      /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
      offsets_type.mag_offset_x = (m_bus.read_8_bits(MAG_OFFSET_X_MSB_ADDR) << 8) |
                                  (m_bus.read_8_bits(MAG_OFFSET_X_LSB_ADDR));
      offsets_type.mag_offset_y = (m_bus.read_8_bits(MAG_OFFSET_Y_MSB_ADDR) << 8) |
                                  (m_bus.read_8_bits(MAG_OFFSET_Y_LSB_ADDR));
      offsets_type.mag_offset_z = (m_bus.read_8_bits(MAG_OFFSET_Z_MSB_ADDR) << 8) |
                                  (m_bus.read_8_bits(MAG_OFFSET_Z_LSB_ADDR));

      /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
         500 dps = +/- 8000 LSB
         250 dps = +/- 4000 LSB
         125 dps = +/- 2000 LSB
         ... where 1 DPS = 16 LSB */
      offsets_type.gyro_offset_x = (m_bus.read_8_bits(GYRO_OFFSET_X_MSB_ADDR) << 8) |
                                   (m_bus.read_8_bits(GYRO_OFFSET_X_LSB_ADDR));
      offsets_type.gyro_offset_y = (m_bus.read_8_bits(GYRO_OFFSET_Y_MSB_ADDR) << 8) |
                                   (m_bus.read_8_bits(GYRO_OFFSET_Y_LSB_ADDR));
      offsets_type.gyro_offset_z = (m_bus.read_8_bits(GYRO_OFFSET_Z_MSB_ADDR) << 8) |
                                   (m_bus.read_8_bits(GYRO_OFFSET_Z_LSB_ADDR));

      /* Accelerometer radius = +/- 1000 LSB */
      offsets_type.accel_radius = (m_bus.read_8_bits(ACCEL_RADIUS_MSB_ADDR) << 8) |
                                  (m_bus.read_8_bits(ACCEL_RADIUS_LSB_ADDR));

      /* Magnetometer radius = +/- 960 LSB */
      offsets_type.mag_radius = (m_bus.read_8_bits(MAG_RADIUS_MSB_ADDR) << 8) |
                                (m_bus.read_8_bits(MAG_RADIUS_LSB_ADDR));

      setMode(lastMode);
      return true;
   }
   return false;
}

[[maybe_unused]] /*!
 *  @brief  Writes an array of calibration values to the sensor's offset
 *  @param  calibData
 *          calibration data
 */
void Adafruit_BNO055::setSensorOffsets(const uint8_t* calibData)
{
   adafruit_bno055_opmode_t lastMode = m_mode;
   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);

   /* Note: Configuration will take place only when user writes to the last
      byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
      Therefore the last byte must be written whenever the user wants to
      changes the configuration. */

   /* A writeLen() would make this much cleaner */
   m_bus.write_8_bits(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
   m_bus.write_8_bits(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
   m_bus.write_8_bits(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
   m_bus.write_8_bits(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
   m_bus.write_8_bits(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
   m_bus.write_8_bits(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

   m_bus.write_8_bits(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
   m_bus.write_8_bits(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
   m_bus.write_8_bits(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
   m_bus.write_8_bits(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
   m_bus.write_8_bits(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
   m_bus.write_8_bits(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

   m_bus.write_8_bits(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
   m_bus.write_8_bits(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
   m_bus.write_8_bits(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
   m_bus.write_8_bits(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
   m_bus.write_8_bits(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
   m_bus.write_8_bits(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

   m_bus.write_8_bits(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
   m_bus.write_8_bits(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

   m_bus.write_8_bits(MAG_RADIUS_LSB_ADDR, calibData[20]);
   m_bus.write_8_bits(MAG_RADIUS_MSB_ADDR, calibData[21]);

   setMode(lastMode);
}

[[maybe_unused]] /*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 *          accel_offset_x = acceleration offset x
 *          accel_offset_y = acceleration offset y
 *          accel_offset_z = acceleration offset z
 *
 *          mag_offset_x   = magnetometer offset x
 *          mag_offset_y   = magnetometer offset y
 *          mag_offset_z   = magnetometer offset z
 *
 *          gyro_offset_x  = gyroscrope offset x
 *          gyro_offset_y  = gyroscrope offset y
 *          gyro_offset_z  = gyroscrope offset z
 */
void Adafruit_BNO055::setSensorOffsets(const adafruit_bno055_offsets_t& offsets_type)
{
   adafruit_bno055_opmode_t lastMode = m_mode;
   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);

   /* Note: Configuration will take place only when user writes to the last
      byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
      Therefore the last byte must be written whenever the user wants to
      changes the configuration. */

   m_bus.write_8_bits(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
   m_bus.write_8_bits(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
   m_bus.write_8_bits(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
   m_bus.write_8_bits(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
   m_bus.write_8_bits(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
   m_bus.write_8_bits(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

   m_bus.write_8_bits(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
   m_bus.write_8_bits(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
   m_bus.write_8_bits(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
   m_bus.write_8_bits(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
   m_bus.write_8_bits(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
   m_bus.write_8_bits(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

   m_bus.write_8_bits(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
   m_bus.write_8_bits(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
   m_bus.write_8_bits(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
   m_bus.write_8_bits(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
   m_bus.write_8_bits(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
   m_bus.write_8_bits(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

   m_bus.write_8_bits(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
   m_bus.write_8_bits(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

   m_bus.write_8_bits(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
   m_bus.write_8_bits(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

   setMode(lastMode);
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool Adafruit_BNO055::isFullyCalibrated()
{
   uint8_t system, gyro, accel, mag;
   getCalibration(&system, &gyro, &accel, &mag);

   switch (m_mode) {
   case OPERATION_MODE_ACCONLY:
      return (accel == 3);
   case OPERATION_MODE_MAGONLY:
      return (mag == 3);
   case OPERATION_MODE_GYRONLY:
   case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
      return (gyro == 3);
   case OPERATION_MODE_ACCMAG:
   case OPERATION_MODE_COMPASS:
      return (accel == 3 && mag == 3);
   case OPERATION_MODE_ACCGYRO:
   case OPERATION_MODE_IMUPLUS:
      return (accel == 3 && gyro == 3);
   case OPERATION_MODE_MAGGYRO:
      return (mag == 3 && gyro == 3);
   default:
      return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
   }
}

[[maybe_unused]] /*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
void Adafruit_BNO055::enterSuspendMode()
{
   adafruit_bno055_opmode_t modeback = m_mode;

   /* Switch to config mode (just in case since this is the default) */
   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);
   m_bus.write_8_bits(BNO055_PWR_MODE_ADDR, 0x02);
   /* Set the requested operating mode (see section 3.3) */
   setMode(modeback);
   gpio::sleep(20ms);
}

[[maybe_unused]] /*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
void Adafruit_BNO055::enterNormalMode()
{
   adafruit_bno055_opmode_t modeback = m_mode;

   /* Switch to config mode (just in case since this is the default) */
   setMode(OPERATION_MODE_CONFIG);
   gpio::sleep(25ms);
   m_bus.write_8_bits(BNO055_PWR_MODE_ADDR, 0x00);
   /* Set the requested operating mode (see section 3.3) */
   setMode(modeback);
   gpio::sleep(20ms);
}
