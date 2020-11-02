#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "protothreads.h"

/*
  Stepper motor control variable setup and method definitions.
*/
#define MOTOR_CMD_SAMPLE_RATE_MS (200);

const int motor_steps_per_rotation = 2048; // @TODO - have separate values for left and right wheel.
float max_motor_speed = 5000, left_speed = 0, right_speed = 0;
AccelStepper leftStepper = AccelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13);
AccelStepper rightStepper = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
long prev_cms_rx_time = 0;

// @TODO - change to incorporate different left and right wheel sizes.
float ros_speed_to_step_speed(float ros_speed, int steps_per_rotation) {
  return (ros_speed / (2 * PI)) * steps_per_rotation;
}

pt pt_motor_cmd_rx_thread_handle;
int motor_cmd_rx_thread(struct pt *pt_handle) {
  // @TODO - Workout speed conversion from command to RPM
  PT_BEGIN(pt_handle);
  while (true) {
    PT_WAIT_UNTIL(pt_handle, Serial.available());
    String cmd = Serial.readStringUntil('\n');
    int sep_idx = cmd.indexOf(",", 0);
    if (sep_idx != -1) {
      left_speed = ros_speed_to_step_speed(cmd.substring(0, sep_idx).toFloat(), motor_steps_per_rotation);
      right_speed = ros_speed_to_step_speed(cmd.substring(sep_idx + 1, cmd.length()).toFloat(), motor_steps_per_rotation);
      leftStepper.setSpeed(-1 * left_speed);
      rightStepper.setSpeed(right_speed);
    }
  }
  PT_END(pt_handle);
}

/*
  BN0-055 IMU variable setup and method definition.
*/
#define I2C_ADD (0x28)
#define BNO055_SAMPLERATE_DELAY_MS (3000)          // Delay between data requests

const double DEG_2_RAD = 0.0174533;
Adafruit_BNO055 bno = Adafruit_BNO055(123, 0x28);          // Create sensor object bno based on Adafruit_BNO055 library
sensors_event_t ang_velocity_data, linear_accel_data;
imu::Quaternion quat;
long prev_imu_tx_time = 0;

void restore_imu_sensor_offsets() {
  /*
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
  bno.getEvent(&ang_velocity_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linear_accel_data, Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat.normalize();
  Serial.print("IMU ");
  Serial.print(quat.x(), 4);  Serial.print(" "); // Print quaternion x
  Serial.print(quat.y(), 4);  Serial.print(" "); // Print quaternion y
  Serial.print(quat.z(), 4);  Serial.print(" ");   // Print quaternion z
  Serial.print(quat.w(), 4);  Serial.print(" "); // Print quaternion w
  Serial.print(ang_velocity_data.gyro.x * DEG_2_RAD, 4); Serial.print(" "); //Print ang vel x
  Serial.print(ang_velocity_data.gyro.y * DEG_2_RAD, 4); Serial.print(" "); //Print ang vel y
  Serial.print(ang_velocity_data.gyro.z * DEG_2_RAD, 4); Serial.print(" "); //Print ang vel z
  Serial.print(linear_accel_data.acceleration.x, 4); Serial.print(" "); //Print linearAccel x
  Serial.print(linear_accel_data.acceleration.y, 4); Serial.print(" "); //Print linearAccel y
  Serial.print(linear_accel_data.acceleration.z, 4); //Print linearAccel z
  Serial.println();
}

pt pt_imu_tx_thread_handle;
int imu_tx_thread(struct pt* pt_handle) {
  PT_BEGIN(pt_handle);
  while (true) {
    PT_WAIT_UNTIL(pt_handle, millis() - prev_imu_tx_time >= BNO055_SAMPLERATE_DELAY_MS);
    print_imu_data();
    prev_imu_tx_time = millis(); // Some delay expecteda as a result of serial transmission.
  }
  PT_END(pt_handle);
}

/*
  Other control variables.
*/
long currentTime = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!bno.begin())                               // Initialize sensor communication
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  restore_imu_sensor_offsets();
  Serial.println("starting...");
  delay(5000);
  bno.setExtCrystalUse(true);                     // Use the crystal on the development board

  // set the speed at 15 rpm:
  leftStepper.setMaxSpeed(max_motor_speed);
  rightStepper.setMaxSpeed(max_motor_speed);
  leftStepper.setSpeed(left_speed);
  rightStepper.setSpeed(right_speed);

  PT_INIT(&pt_imu_tx_thread_handle);
  PT_INIT(&pt_motor_cmd_rx_thread_handle);

  currentTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  rightStepper.runSpeed();
  leftStepper.runSpeed();
  PT_SCHEDULE(imu_tx_thread(&pt_imu_tx_thread_handle));
  PT_SCHEDULE(motor_cmd_rx_thread(&pt_motor_cmd_rx_thread_handle));
}
