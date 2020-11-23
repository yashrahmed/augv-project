#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <NeoSWSerial.h>
#include "protothreads.h"

#define IMU_I2C_ADD (0x28)
#define SLAVE_CTRL_I2C_ADD (0x04)

// STEPPER MOTOR SETUP -------------
#define MOTOR_UPDATE_FREQ_MS (50)

union {
  float val;
  byte bval[4];
} floatAsBytes;

float motor_state[4] = {0, 0, 0, 0};

void send_motor_cmd_to_slave(float left_speed, float right_speed) {
  // Command is 2 floats for left and right motor
  Wire.beginTransmission(SLAVE_CTRL_I2C_ADD);
  floatAsBytes.val = left_speed;
  Wire.write(floatAsBytes.bval, 4);
  floatAsBytes.val = right_speed;
  Wire.write(floatAsBytes.bval, 4);
  Wire.endTransmission();
}

void update_motor_state_from_slave() {
  // expected response is in the form of four float values
  Wire.requestFrom(SLAVE_CTRL_I2C_ADD, 16);
  while (Wire.available()) {
    for (int value_ct = 0; value_ct < 4; value_ct++) {
      for (int byte_no = 0; byte_no < 4; byte_no++) {
        floatAsBytes.bval[byte_no] = Wire.read();
      }
      motor_state[value_ct] = floatAsBytes.val;
    }
  }
}

void print_motor_state() {
  update_motor_state_from_slave();
  Serial.print("MOT,");
  for (int value_ct = 0; value_ct < 3; value_ct++) {
    Serial.print(motor_state[value_ct], 3);
    Serial.print(",");
  }
  Serial.println(motor_state[3], 3); // print last state value
}

pt pt_motor_reader;
int motor_read_thread(struct pt *pt_handle) {
  PT_BEGIN(pt_handle);
  while (true) {
    print_motor_state();
    PT_SLEEP(pt_handle, MOTOR_UPDATE_FREQ_MS);
  }
  PT_END(pt_handle);
}

long last_write_time = 0;
pt pt_motor_writer;
int motor_write_thread(struct pt *pt_handle) {
  PT_BEGIN(pt_handle);
  while (true) {
    PT_WAIT_UNTIL(pt_handle, Serial.available() && ((millis() - last_write_time) > MOTOR_UPDATE_FREQ_MS));
    String cmd = Serial.readStringUntil('\n');
    int idx = cmd.indexOf(",");
    float left_speed = cmd.substring(0, idx).toFloat();
    float right_speed = cmd.substring(idx + 1, cmd.length()).toFloat();
    send_motor_cmd_to_slave(left_speed, right_speed);
    last_write_time = millis();
  }
  PT_END(pt_handle);
}

// IMU SETUP -------------

#define BNO055_SAMPLERATE_DELAY_MS (100)          // Delay between data requests

const double DEG_2_RAD = 0.0174533;
Adafruit_BNO055 bno = Adafruit_BNO055(123, IMU_I2C_ADD);          // Create sensor object bno based on Adafruit_BNO055 library
sensors_event_t ang_velocity_data, linear_accel_data;
imu::Quaternion quat;
long prev_imu_tx_time = 0;

void restore_imu_sensor_offsets() {
  /*
    Accelerometer: -26 -9 -24
    Gyro: -1 -1 -1
    Mag: -39 -421 -326
    Accel Radius: 1000
    Mag Radius: 833

  */
  bno.setMode(bno.OPERATION_MODE_CONFIG);
  adafruit_bno055_offsets_t calibData;
  calibData.accel_offset_x = -26;
  calibData.accel_offset_y = -9;
  calibData.accel_offset_z = -24;

  calibData.gyro_offset_x = -1;
  calibData.gyro_offset_y = -1;
  calibData.gyro_offset_z = -1;

  calibData.mag_offset_x = -39;
  calibData.mag_offset_y = -421;
  calibData.mag_offset_z = -326;

  calibData.accel_radius = 1000;
  calibData.mag_radius = 833;

  bno.setSensorOffsets(calibData);
  bno.setMode(bno.OPERATION_MODE_NDOF);
  //bno.setMode(bno.OPERATION_MODE_IMUPLUS);
}

void print_imu_data() {
  quat = bno.getQuat();           // Request quaternion data from BNO055
  bno.getEvent(&ang_velocity_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linear_accel_data, Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat.normalize();
  Serial.print("IMU,");
  Serial.print(quat.x(), 4);  Serial.print(","); // Print quaternion x
  Serial.print(quat.y(), 4);  Serial.print(","); // Print quaternion y
  Serial.print(quat.z(), 4);  Serial.print(",");   // Print quaternion z
  Serial.print(quat.w(), 4);  Serial.print(","); // Print quaternion w
  Serial.print(ang_velocity_data.gyro.x * DEG_2_RAD, 4); Serial.print(","); //Print ang vel x
  Serial.print(ang_velocity_data.gyro.y * DEG_2_RAD, 4); Serial.print(","); //Print ang vel y
  Serial.print(ang_velocity_data.gyro.z * DEG_2_RAD, 4); Serial.print(","); //Print ang vel z
  Serial.print(linear_accel_data.acceleration.x, 4); Serial.print(","); //Print linearAccel x
  Serial.print(linear_accel_data.acceleration.y, 4); Serial.print(","); //Print linearAccel y
  Serial.print(linear_accel_data.acceleration.z, 4); //Print linearAccel z
  Serial.println();
}

pt pt_imu;
int imu_read_thread(struct pt *pt_handle) {
  PT_BEGIN(pt_handle);
  while (true) {
    print_imu_data();
    PT_SLEEP(pt_handle, BNO055_SAMPLERATE_DELAY_MS);
  }
  PT_END(pt_handle);
}

// GPS SETUP -------------
#define GPS_SAMPLE_DELAY_MS (100)

const int RXPin = 4, TXPin = 3;
const int GPS_BAUD_RATE = 9600;
long prev_gps_check_time = 0;

TinyGPSPlus gps;
NeoSWSerial gpsSerial(RXPin, TXPin);

void print_gps_data() {
  // message format is [GPS lat lon alt hdop course]
  if (gps.location.isValid() && gps.hdop.isValid() && gps.location.isUpdated()) {
    Serial.print("GPS,");
    Serial.print(gps.location.lat(), 8); Serial.print(",");
    Serial.print(gps.location.lng(), 8); Serial.print(",");
    Serial.print(gps.altitude.meters(), 8); Serial.print(",");
    Serial.print(gps.hdop.hdop(), 8); Serial.print(",");
    Serial.print(gps.course.deg(), 8); Serial.println();
  }
}

pt pt_gps;
int gps_read_thread(struct pt *pt_handle) {
  PT_BEGIN(pt_handle);

  while (gpsSerial.available() > 0 && ((millis() - prev_gps_check_time) > GPS_SAMPLE_DELAY_MS)) {
    gps.encode(gpsSerial.read());
    print_gps_data();
    PT_YIELD(pt_handle);
  }
  PT_END(pt_handle);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  if (!bno.begin())                               // Initialize sensor communication
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  restore_imu_sensor_offsets();
  Serial.println("starting...");
  delay(2000);
  bno.setExtCrystalUse(true);                     // Use the crystal on the development board
  gpsSerial.begin(GPS_BAUD_RATE);
  delay(2000);
  PT_INIT(&pt_gps);
  PT_INIT(&pt_imu);
  PT_INIT(&pt_motor_reader);
  PT_INIT(&pt_motor_writer);
}

void loop() {
  // put your main code here, to run repeatedly:
  PT_SCHEDULE(gps_read_thread(&pt_gps));
  PT_SCHEDULE(imu_read_thread(&pt_imu));
  PT_SCHEDULE(motor_read_thread(&pt_motor_reader));
  PT_SCHEDULE(motor_write_thread(&pt_motor_writer));
}
