  
struct PID_parametres {
  float Kp;
  float Ki;
  float Kd;
};

struct regulator_parametres {
  struct PID_parametres constants;
  float previous_error;
  float integral;
  float setpoint;
  float regulator_out;
};

struct motor_parametres {
  unsigned char pin;
  unsigned char value;
};

struct motor {
  struct motor_parametres left;
  struct motor_parametres right;
};

struct regulator_parametres regulator = {0.1, 0.2, 0.3, 0, 0, 0, 0};
struct motor motor = {2, 255, 3, 255};

float dt = 0.01;
float angle = 0;
  
void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);

  pinMode(motor.left.pin, OUTPUT);
  pinMode(motor.right.pin, OUTPUT);
}

float PID_controller(float input_angle);
  
void loop() {

  analogWrite(motor.left.pin, motor.left.value);
  analogWrite(motor.right.pin, motor.right.value);

  float regulator_out = PID_controller(angle);
}


float PID_controller(float input_angle) {
  /*START DELTA T TIMING*/
  unsigned long my_time = millis();
  
  /*PID CONTROLLER*/
  float error = regulator.setpoint - input_angle;
  regulator.integral = regulator.integral + error*dt;
  float derivative = (error - regulator.previous_error)/dt;
  
  /*WAIT FOR DELTA T*/
  while(millis() - my_time < dt*1000);
  
  regulator.previous_error = error;
  return (regulator.constants.Kp*error + regulator.constants.Ki*
      regulator.integral + regulator.constants.Kd*derivative);
}
