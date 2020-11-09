#include <Wire.h>
#include <AccelStepper.h>

#define IMU_I2C_ADD (0x28)
#define MOTOR_CTRL_I2C_ADD (0x04)

AccelStepper leftStepper = AccelStepper(AccelStepper::FULL4WIRE, 10, 11, 12, 13);
AccelStepper rightStepper = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);

const float left_wheel_scale_factor = 1.0023;
const float max_motor_speed = 5000;
const float motor_steps_per_rotation = 2048; // @TODO - have a multiplier for differing valuefor left and right wheel.
float left_step_speed = 0, left_ros_speed = 0, right_step_speed = 0, right_ros_speed = 0, left_ros_angle = 0, right_ros_angle = 0;

union {
  float val;
  byte bval[4];
} floatAsBytes;

float read_float_from_bytes(int bytes) {
  for (int i = 0; i < bytes; i++) {
    floatAsBytes.bval[i] = Wire.read();
  }
  return floatAsBytes.val;
}

float write_float_to_bytes(float x) {
  floatAsBytes.val = x;
  for (int i = 0; i < 4; i++) {
    Wire.write(floatAsBytes.bval[i]);
  }
}

// parameter k is used to incorporate different left and right wheel size.
// k = 1 for right wheel and 1/left_wheel_scale_factor for the other.
float ros_speed_to_step_speed(float ros_speed, float k) {
  return ((ros_speed / k) / (2 * PI)) * motor_steps_per_rotation;
}
// parameter k is used to incorporate different left and right wheel size.
// k = 1 for right wheel and 1/left_wheel_scale_factor for the other.
float steps_to_ros_angle(long steps, float k) {
  return 2 * PI * (steps / motor_steps_per_rotation) * k;
}

void set_motor_speed_on_cmd() {
  left_step_speed = ros_speed_to_step_speed(left_ros_speed, left_wheel_scale_factor);
  right_step_speed = ros_speed_to_step_speed(right_ros_speed, 1.0);
  // -1 direction multiplier is applied as the motors are mirrored relative to each other.
  // This makes the positive direction of one the oppsite of the other.
  leftStepper.setSpeed(-1 * left_step_speed);
  rightStepper.setSpeed(right_step_speed);
}

void receiveEvent(int bytes) {
  left_ros_speed = read_float_from_bytes(4);
  right_ros_speed = read_float_from_bytes(4);
  set_motor_speed_on_cmd();
}

void requestEvent() {
  write_float_to_bytes(left_ros_speed);
  write_float_to_bytes(right_ros_speed);
  write_float_to_bytes(-steps_to_ros_angle(leftStepper.currentPosition(), left_wheel_scale_factor));
  write_float_to_bytes(steps_to_ros_angle(rightStepper.currentPosition(), 1.0));
}

void setup() {
  // put your setup code here, to run once:
  leftStepper.setMaxSpeed(max_motor_speed);
  rightStepper.setMaxSpeed(max_motor_speed);
  leftStepper.setSpeed(left_step_speed);
  rightStepper.setSpeed(right_step_speed);

  Wire.begin(MOTOR_CTRL_I2C_ADD);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {
  rightStepper.runSpeed();
  leftStepper.runSpeed();
}
