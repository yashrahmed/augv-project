#include <PIDController.h>

#define FWD (1)
#define OFF (0)
#define BWD (-1)
// Only Pins 2 and 3 can be used for Interrupts
#define ENCA (2)
#define ENCB (8)

////// BEGIN MOTOR CLASS
class Motor {
  private:
    int dirPin1;
    int dirPin2;
    int spdPin;
    int encA;
    int encB;
    volatile int curr_pulses = 0;
    int prev_pulses = 0;
    unsigned long last_sample_time = 0;
    float velocity;

  public:
    SmartMotor() {}
    void init_pins(int spdPin, int dirPin1, int dirPin2, int encA, int encB);
    float calc_velocity();
    void interrupt_handler();
    int get_interrupt_pin();
    float get_velocity();
    void set_power(int power);
};

void Motor::init_pins(int spdPin, int dirPin1, int dirPin2, int encA, int encB) {
  this->dirPin1 = dirPin1;
  this->dirPin2 = dirPin2;
  this->spdPin = spdPin;
  this->encA = encA;
  this->encB = encB;
  pinMode(this->spdPin, OUTPUT);
  pinMode(this->dirPin1, OUTPUT);
  pinMode(this->dirPin2, OUTPUT);
  pinMode(this->encA, INPUT);
  pinMode(this->encB, INPUT);
}

float Motor::calc_velocity() {
  /*
    Sample pulses to measure velocity.
    Multiply by 1.0 to avoid int casting.
  */
  unsigned long curr_time = millis();
  if (curr_time - this->last_sample_time >= 100) {
    velocity = (((this->curr_pulses - this->prev_pulses) * 1.0) / (curr_time - this->last_sample_time)) * 1000;
    this->last_sample_time = curr_time;
    this->prev_pulses = this->curr_pulses;
  }
}

void Motor::interrupt_handler() {
  int b = digitalRead(this->encB);
  if (b > 0) {
    this->curr_pulses++;
  }
  else {
    this->curr_pulses--;
  }
}

int Motor::get_interrupt_pin() {
  return this-> encA;
}

float Motor::get_velocity() {
  return this->velocity;
}

void Motor::set_power(int power) {
  int direction = (power < 0) ? BWD : FWD;
  analogWrite(this->spdPin, constrain(abs(power), 0, 255));
  switch (direction) {
    case FWD:
      digitalWrite(this->dirPin1, HIGH);
      digitalWrite(this->dirPin2, LOW);
      break;
    case BWD:
      digitalWrite(this->dirPin1, LOW);
      digitalWrite(this->dirPin2, HIGH);
      break;
    default:
      digitalWrite(this->dirPin1, LOW);
      digitalWrite(this->dirPin2, LOW);
  }
}

////// END MOTOR CLASS


////// BEGIN PID_MOTOR_VELOCITY_CONTROLLER CLASS
class PIDMotorVelocityController {
  private:
    //https://github.com/DonnyCraft1/PIDArduino
    PIDController pid;
    int setpoint;
    int curr_output;
  public:
    PIDMotorVelocityController() {}
    void setup(float kp, float ki, float kd);
    void set_target(int target);
    int get_control_output(float velocity);
    int get_target();
};

void PIDMotorVelocityController::setup(float kp, float ki, float kd) {
  PIDController pid = PIDController();
  pid.begin();          // initialize the PID instance
  pid.tune(kp, ki, kd);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(-255, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup
  this->pid = pid;
}

void PIDMotorVelocityController::set_target(int target) {
  this->setpoint = target;
  this->pid.setpoint(target);  // The "goal" the PID controller tries to "reach"
}

int PIDMotorVelocityController::get_control_output(float velocity) {

  /*
    For velocity control via PID the output of the controller must be accumulated over time.
    For smooth operation ensure that params are set to a fraction of the velocity error.

    Note - For an actual implementation a hard limit on curr_output is needed as it can accumulate infinitely and overflow.
  */
  curr_output += pid.compute(velocity);;
  curr_output = constrain(curr_output, -500, 500);
  return curr_output;
}

int PIDMotorVelocityController::get_target() {
  return this->setpoint;
}

////// END PID_MOTOR_VELOCITY_CONTROLLER CLASS

Motor left_motor, right_motor;
PIDMotorVelocityController left_controller, right_controller;

// Necessary as ISR must be static.
void left_motor_encoder_cb() {
  left_motor.interrupt_handler();
}

void right_motor_encoder_cb() {
  right_motor.interrupt_handler();
}


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  /*
    Motor polarity must match encoder direction i.e. FWD direction must increase encoder count and vice versa.

    left motor FWD = anti-CW
    right motor FWD = anti-CW
  */
  left_motor.init_pins(5, 6, 7, 2, 8);
  right_motor.init_pins(11, 12, 13, 3, 4);

  attachInterrupt(digitalPinToInterrupt(left_motor.get_interrupt_pin()), left_motor_encoder_cb, RISING);
  attachInterrupt(digitalPinToInterrupt(right_motor.get_interrupt_pin()), right_motor_encoder_cb, RISING);

  left_controller.setup(0.25, 0.0, 0.0);
  right_controller.setup(0.25, 0.0, 0.0);

  Serial.println("STARTING");
  delay(10);
}

void loop() {
  while (Serial.available()) {
    right_controller.set_target(Serial.readStringUntil(' ').toInt());
    left_controller.set_target(Serial.readStringUntil('\n').toInt());
  }

  right_motor.calc_velocity();
  float right_velocity = right_motor.get_velocity();
  int right_output = right_controller.get_control_output(right_velocity);
  right_motor.set_power(right_output);

  left_motor.calc_velocity();
  float left_velocity = left_motor.get_velocity();
  int left_output = left_controller.get_control_output(left_velocity);
  left_motor.set_power(left_output);

  //  Serial.print(right_controller.get_target()); Serial.print(' '); Serial.print(velocity); Serial.print(' '); Serial.println(output);
  delay(20);
}
