//----------------------DEFINE----------------------
#define SERIAL_BAUD_RATE 9600
#define DIRECTION_SWITCH_DELAY 20
#define MTR1_PIN_FWD 15
#define MTR1_PIN_REV 14
#define MTR1_PIN_PWM 5
#define MTR2_PIN_FWD 16
#define MTR2_PIN_REV 17
#define MTR2_PIN_PWM 6
#define MOTOR_TEST_SPEED 100
#define DIV 1
#include "TRSensors.h"
#define NUM_SENSORS 5

//----------------------END-DEFINE----------------------


//----------------------MOTOR----------------------
enum direction{FORWARD, REVERSE, DISABLED};
static int id_counter=0;
class Motor{
  public:
    const int GPIO1, GPIO2, PWM;
    int id;
  
  Motor( int pin_fwd, int pin_rev, int pin_pwm) 
    : GPIO1(pin_fwd), GPIO2(pin_rev), PWM(pin_pwm){
      //lista inicjalizacyjna
      id = getUniqueId();
  }

  int getUniqueId(){return ++id_counter;}

  void initialize(){
    pinMode(GPIO1, OUTPUT);
    digitalWrite(GPIO1, LOW);
    pinMode(GPIO2, OUTPUT);
    digitalWrite(GPIO2, LOW);
    pinMode(PWM, OUTPUT);
    analogWrite(PWM, 0);
    set_direction(FORWARD);
    // set_speed(SPEED_MOTOR_TEST);
  }

  void set_speed(unsigned char value){analogWrite(PWM, value);}
  
  void set_direction(enum direction dir){
    switch(dir){
      case FORWARD:
        digitalWrite(GPIO2, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(GPIO1, HIGH); 
        break; 
      
      case REVERSE:
        digitalWrite(GPIO1, LOW);
        delay(DIRECTION_SWITCH_DELAY);
        digitalWrite(GPIO2, HIGH); 
        break;
      
      case DISABLED:
        digitalWrite(GPIO1, LOW);
        digitalWrite(GPIO2, LOW); 
        break;
    }
  }
  
  void stop(){
    set_direction(DISABLED);
    set_speed(0);
  }
};

Motor motor1 = Motor(MTR1_PIN_FWD, MTR1_PIN_REV, MTR1_PIN_PWM);
Motor motor2 = Motor(MTR2_PIN_FWD, MTR2_PIN_REV, MTR2_PIN_PWM);
//----------------------END-MOTOR----------------------


//----------------------VEHICLE----------------------
class Vehicle{
  public:
    Motor &motor_l, &motor_r;
    int t_delta;

  Vehicle( Motor& m_l, Motor& m_r)
    : motor_l(m_l), motor_r(m_r){
      t_delta=1000;
      motor_l.set_speed(0);
      motor_r.set_speed(0);
  }

  void set_speed(int speed){
      motor_r.set_speed(speed);
      motor_l.set_speed(speed);
  }

  void set_speed(int left, int right){
      motor_r.set_speed(left);
      motor_l.set_speed(right);
  }

  void set_direction(direction dir){
      motor_l.set_direction(dir);
      motor_r.set_direction(dir);
    }

  void stop(){
    set_speed(0);
    motor_l.set_direction(DISABLED);
    motor_r.set_direction(DISABLED);
  }

  void test_motors() {
    motor_l.set_direction(FORWARD);
    motor_l.set_speed(150);
    delay(2000);
    motor_l.set_speed(0);

    motor_r.set_direction(FORWARD);
    motor_r.set_speed(150);
    delay(2000);
    motor_r.set_speed(0);
  }
};

Vehicle vehicle = Vehicle(motor1, motor2);
//----------------------END-VEHICLE----------------------


//----------------------SENSOR----------------------


TRSensors trs = TRSensors();
unsigned int sensor_values[NUM_SENSORS];
//----------------------END-SENSOR----------------------

//----------------------SETUP----------------------
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  motor1.initialize();
  motor2.initialize();
  delay(100);
  calibration();
  vehicle.set_direction(FORWARD);
}
//----------------------END-SETUP----------------------


//----------------------LOOP----------------------
void loop() {
  unsigned int position = trs.readLine(sensor_values);
  if (position > 2000 && position <3000) {
    vehicle.set_speed(MOTOR_TEST_SPEED);
    }
  else correct_course(position);
}
//----------------------END-LOOP----------------------


//----------------------FUNCTIONS----------------------
void calibration(){
  for (int i=0; i<400; i++){
    trs.calibrate();
    // print_sensors();
    delay(25);
  }
}
//----------------------END-FUNCTIONS----------------------


//----------------------PID----------------------
double Kp = 0.049;
double Ki = 0;
double Kd = 0.09;

double error = 0.0;
double previousError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double output = 0.0;

void correct_course(int position) {
    const int targetPosition = 2500; 
    error = position - targetPosition;
    integral += error;
    derivative = error - previousError;
    output = Kp * error + Ki * integral + Kd * derivative;

    int turn = output / DIV;
    int leftSpeed = constrain(MOTOR_TEST_SPEED + turn, 0, 255);
    int rightSpeed = constrain(MOTOR_TEST_SPEED - turn, 0, 255);
    vehicle.set_speed(leftSpeed, rightSpeed);
    previousError = error;
}
//----------------------END-PID----------------------

