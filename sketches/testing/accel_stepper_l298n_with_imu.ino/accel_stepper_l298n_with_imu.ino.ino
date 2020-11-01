#include <Stepper.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
int leftSteps = 0, rightSteps = 0, steps = 0, dir = -1;
long startTime = 0, lastImuTime = 0;
bool movingFwd = true;

#define I2C_ADD (0x28)
#define BNO055_SAMPLERATE_DELAY_MS (2500)          // Delay between data requests

const double DEG_2_RAD = 0.0174533;
Adafruit_BNO055 bno = Adafruit_BNO055(123, 0x28);          // Create sensor object bno based on Adafruit_BNO055 library
sensors_event_t angVelocityData, linearAccelData;
imu::Quaternion quat;

// for your motor

// initialize the stepper library on pins 8 through 11:
// 8,10,9,11 for unipolar mode.
/*
   OUT 1 - Blue
   OUT 2 - Yellow
   OUT 3 - Pink
   OUT 4 - Orange
   ENA and ENB must both be high...

*/
//Stepper rightStepper(stepsPerRevolution, 10, 11, 12, 13);
//Stepper leftStepper(stepsPerRevolution, 6, 7, 8, 9);

AccelStepper leftStepper = AccelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13);
AccelStepper rightStepper = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);



void restore_sensor_offsets() {
  /*
    Accelerometer: -26 32 -15
    Gyro: 0 -1 -1
    Mag: -815 -274 -152
    Accel Radius: 1000
    Mag Radius: 784

    ON-TABLE?
    Accelerometer: 5 10 -18
    Gyro: -2 -1 -1
    Mag: -734 -4 -847
    Accel Radius: 1000
    Mag Radius: 737


  */
  bno.setMode(bno.OPERATION_MODE_CONFIG);
  adafruit_bno055_offsets_t calibData;
  calibData.accel_offset_x = 5;
  calibData.accel_offset_y = 10;
  calibData.accel_offset_z = 18;

  calibData.gyro_offset_x = -2;
  calibData.gyro_offset_y = -1;
  calibData.gyro_offset_z = -1;

  calibData.mag_offset_x = -734;
  calibData.mag_offset_y = -4;
  calibData.mag_offset_z = -847;

  calibData.accel_radius = 1000;
  calibData.mag_radius = 737;

  bno.setSensorOffsets(calibData);
  //bno.setMode(bno.OPERATION_MODE_NDOF);
  bno.setMode(bno.OPERATION_MODE_IMUPLUS);
}

void print_imu_data() {
  quat = bno.getQuat();           // Request quaternion data from BNO055
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat.normalize();
  Serial.print("IMU ");
  Serial.print(quat.x(), 4);  Serial.print(" "); // Print quaternion x
  Serial.print(quat.y(), 4);  Serial.print(" "); // Print quaternion y
  Serial.print(quat.z(), 4);  Serial.print(" ");   // Print quaternion z
  Serial.print(quat.w(), 4);  Serial.print(" "); // Print quaternion w
  Serial.print(angVelocityData.gyro.x * DEG_2_RAD, 4); Serial.print(" "); //Print ang vel x
  Serial.print(angVelocityData.gyro.y * DEG_2_RAD, 4); Serial.print(" "); //Print ang vel y
  Serial.print(angVelocityData.gyro.z * DEG_2_RAD, 4); Serial.print(" "); //Print ang vel z
  Serial.print(linearAccelData.acceleration.x, 4); Serial.print(" "); //Print linearAccel x
  Serial.print(linearAccelData.acceleration.y, 4); Serial.print(" "); //Print linearAccel y
  Serial.print(linearAccelData.acceleration.z, 4); //Print linearAccel z
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  if (!bno.begin())                               // Initialize sensor communication
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  restore_sensor_offsets();
  Serial.println("starting...");
  delay(5000);
  bno.setExtCrystalUse(true);                     // Use the crystal on the development board

  // set the speed at 15 rpm:
  leftStepper.setMaxSpeed(5000);
  rightStepper.setMaxSpeed(5000);
  leftStepper.setSpeed(-512);
  rightStepper.setSpeed(522.5);
  startTime = millis();
}

void loop() {
  if (millis() - lastImuTime > BNO055_SAMPLERATE_DELAY_MS) {
    print_imu_data();
    lastImuTime = millis();
  }

  if (millis() - startTime > 18000) {
    return;
  }

  leftStepper.runSpeed();
  rightStepper.runSpeed();
}
