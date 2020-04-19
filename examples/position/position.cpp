#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <iostream>

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // how often to read data from the board
uint16_t PRINT_DELAY_MS = 500;            // how often to print the data
uint16_t printCount = 0;                  // counter to avoid printing every 10MS sample

// velocity = accel*dt (dt in seconds)
// position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

using namespace std::chrono_literals;
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
   } else {
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
   if (!bno.begin()) {
      std::cout << "No BNO055 detected" << std::endl;
      return 1;
   }

   gpio::sleep(1000ms);

   while (true) {
      //
      const auto tStart = std::chrono::high_resolution_clock::now();
      sensors_event_t orientationData, linearAccelData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

      xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
      yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

      // velocity of sensor in the direction it's facing
      headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x /
                   cos(DEG_2_RAD * orientationData.orientation.x);

      if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
         // enough iterations have passed that we can print the latest data
         std::cout << "Heading: " << orientationData.orientation.x << " ";
         std::cout << "Position: " << xPos << " , " << yPos << std::endl;
         std::cout << "Speed: " << headingVel << std::endl;
         std::cout << "-------" << std::endl;

         printCount = 0;
      } else {
         printCount = printCount + 1;
      }

      while ((std::chrono::high_resolution_clock::now() - tStart).count() <
             (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
         // poll until the next sample is ready
      }
   }
}
