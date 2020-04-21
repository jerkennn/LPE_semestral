/*PID PARAMETERS*/
#define Kp 0.7    //proportional coefficient
#define Ki 0.2    //integral coefficient
#define Kd 0.2    //derivative coefficient
  
/*GLOBAL PID VARIABLES*/
float previous_error = 0;
float integral = 0;
float setpoint = 0;

float dt = 0.01;
float angle = 0;
float regulator_out = 0;

int left_motor = 2;
int right_motor = 3;
int left_motor_val = 238;
int right_motor_val = 255;
  
void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);

  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);
}

float PID_controller(float angle);
  
void loop() {

  analogWrite(left_motor, left_motor_val);
  analogWrite(right_motor, right_motor_val);

  regulator_out = PID_controller(angle);
}


float PID_controller(float angle) {
  /*START DELTA T TIMING*/
  unsigned long my_time = millis();
  
  /*PID CONTROLLER*/
  float error = setpoint - angle;
  integral = integral + error*dt;
  float derivative = (error - previous_error)/dt;
  float output = (Kp*error + Ki*integral + Kd*derivative);
  previous_error = error;
  
  /*WAIT FOR DELTA T*/
  while(millis() - my_time < dt*1000);
  
  return output;
}
