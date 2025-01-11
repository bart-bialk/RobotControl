#define DIRECTION_SWITCH_DELAY 200
#define MEASUREMENT_TIME_MS 1000
#define PIN_ENCODER_INTERRUPT 2
#define TEST1_SPEED 100

enum Direction{
    FORWARD,
    REVERSE,
    DISABLED
  };

class Motor{
    int pin_fwd;
    int pin_rev;
    int pin_pwm;
    int counter;

    public: Motor(int pin_fwd, int pin_rev, int pin_pwm){
        this->pin_fwd = pin_fwd;
        this->pin_rev = pin_rev;
        this->pin_pwm = pin_pwm;
        this->counter = 0;
      }

    void initialize(){
        pinMode(pin_fwd, OUTPUT);
        digitalWrite(pin_fwd, LOW);
        pinMode(pin_rev, OUTPUT);
        digitalWrite(pin_rev, LOW);
        pinMode(pin_pwm, OUTPUT);
        analogWrite(pin_pwm, 0);
    }

    void stop(){
      set_direction(DISABLED);
      set_speed(0);
    }

    void set_direction(Direction direction){
        if(direction == FORWARD)
        {
          digitalWrite(pin_rev, LOW);
          delay(DIRECTION_SWITCH_DELAY);
          digitalWrite(pin_fwd, HIGH);
        }
        else if (direction == REVERSE)
        {
          digitalWrite(pin_fwd, LOW);
          delay(DIRECTION_SWITCH_DELAY);
          digitalWrite(pin_rev, HIGH);
        }
        else if (direction == DISABLED) {
          digitalWrite(pin_rev, LOW);
          delay(DIRECTION_SWITCH_DELAY);
          digitalWrite(pin_fwd, LOW);
        }
    }

    void set_speed(double velocity){
      if(velocity < 0 ){
        analogWrite(pin_pwm, 0);
      }
      else if(velocity > 255){
        analogWrite(pin_pwm, 255);
      }
      else{
        analogWrite(pin_pwm, velocity);
      }
    }

    void reset_counter(){
      this->counter = 0;
    }

    void increment(){
      this->counter++;
    }

    int get_counter(){
      return this->counter;
    }

};

auto motor1 = Motor(14, 15, 5);
auto motor2 = Motor(16, 17, 6);
String command;
int counter;

void handler1(){
  motor1.increment();
}

void handler2(){
  motor2.increment();
}

void setup() {
  Serial.begin(9600);
  motor1.initialize();
  motor2.initialize();
  attachInterrupt(digitalPinToInterrupt(2), handler1, RISING);
  attachInterrupt(digitalPinToInterrupt(3), handler2, RISING);
  motor1.reset_counter();
  motor2.reset_counter();
}

void loop() {
  command = Serial.readString();
  auto value = stoi(command.substr(s.find(" ") + 1));
  // M1 = 0.20161764705882348 * x + 61.40441176470589
  // M2 = 0.45191176470588235 * x + 66.4779411764706
  if(command.substr(0, s.find(" ") == "M")){
    if(value < 0){
      motor1.set_direction(REVERSE);
      motor2.set_direction(REVERSE);
      motor1.set_speed(abs((value-61.40441176470589)/0.20161764705882348));
      motor2.set_speed(abs((value-66.4779411764706)/0.45191176470588235));
      while(motor1.get_counter() != value && motor2.get_counter() != value){
          
      }
    }
    else if(value > 0){
      motor1.set_direction(FORWARD);
      motor2.set_direction(FORWARD);
      motor1.set_speed((value-61.40441176470589)/0.20161764705882348);
      motor2.set_speed((value-66.4779411764706)/0.45191176470588235));
      while(motor1.get_counter() != value && motor2.get_counter() != value){
          
      }
    }
  }
  else if(command.substr(0, s.find(" ") == "T"){
    auto value = stoi(command.substr(s.find(" ") + 1));
    if(value < 0){
      motor1.set_direction(REVERSE);
      motor2.set_direction(FORWARD);
      motor1.set_speed(abs((value-61.40441176470589)/0.20161764705882348));
      motor2.set_speed(abs((value-66.4779411764706)/0.45191176470588235));
      while(motor1.get_counter() != value && motor2.get_counter() != value){

      }
    }
    else if(value > 0){
      motor1.set_direction(FORWARD);
      motor2.set_direction(REVERSE);
      motor1.set_speed((value-61.40441176470589)/0.20161764705882348);
      motor2.set_speed((value-66.4779411764706)/0.45191176470588235));
      while(motor1.get_counter() != value && motor2.get_counter() != value){
          
      }
    }
  }
  else{
    Serial.println("Invalid command!", DEC);
  }
  Serial.println(motor1.get_counter(), DEC);
  Serial.println(motor2.get_counter(), DEC);
}



