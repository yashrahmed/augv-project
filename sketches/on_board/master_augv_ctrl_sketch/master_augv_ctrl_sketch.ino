#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define IMU_I2C_ADD (0x28)
#define SLAVE_CTRL_I2C_ADD (0x04)

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

#define BNO055_SAMPLERATE_DELAY_MS (50)          // Delay between data requests

const double DEG_2_RAD = 0.0174533;
Adafruit_BNO055 bno = Adafruit_BNO055(123, IMU_I2C_ADD);          // Create sensor object bno based on Adafruit_BNO055 library
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
}

void loop() {
  // put your main code here, to run repeatedly:

  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int idx = cmd.indexOf(",");
    float left_speed = cmd.substring(0, idx).toFloat();
    float right_speed = cmd.substring(idx + 1, cmd.length()).toFloat();
    send_motor_cmd_to_slave(left_speed, right_speed);
  }

  print_motor_state();
  print_imu_data();
  delay(100);
}
