/*
|**********************************************************************
* Project           : Laboratories of Industrial Electronics and Sensors
*                     Semestral project: Robot car
*
* Program name      : Arduino_mega2560.ino
*
* Author            : Vojtech Kozel (kozelvo1)
*
* Date created      : 20200420
*
* Purpose           : Main board code.
*
* Revision History  :
*
* Date        Author      Rev    Revision (Date in YYYYMMDD format) 
* 20200420    kozelvo1    1      Foundation of the project.
* 20200421    kozelvo1    2      Basic form, PID controller implementation.
* 20200422    kozelvo1    3      Variables changed into structs.
*                                Added serial communication.
*
|**********************************************************************
*/

// Reguator structures.
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

// Motors structures.
struct motor_parametres {
  unsigned char pin;
  unsigned char value;
};

struct motor {
  struct motor_parametres left;
  struct motor_parametres right;
};

// Serial communication structures.
struct bluetooth {
  float msg_value;
};

struct gyro {
  char msg[10];
  bool initialized;
  float gyro_angle;
};

struct serial_com {
  struct bluetooth bluetooth;
  struct gyro gyro;
};


struct regulator_parametres regulator = {0.1, 0.2, 0.3, 0, 0, 0, 0};
struct motor motor = {2, 255, 3, 255};
struct serial_com communication;

float dt = 0.1;
float angle = 0;
  
void setup() {
  /* Setup communication, processor pins and variables. */
  Serial.begin(115200);
  Serial2.begin(9600);
  Serial3.begin(9600);

  pinMode(motor.left.pin, OUTPUT);
  pinMode(motor.right.pin, OUTPUT);
  
  communication.gyro.initialized = false;
}

float PID_controller(float input_angle);
void gyro_communication(); 
  
void loop() {
  /* Main loop. */

  analogWrite(motor.left.pin, motor.left.value);
  analogWrite(motor.right.pin, motor.right.value);

  float regulator_out = PID_controller(angle);
  gyro_communication();

  delay(100);
}


float PID_controller(float input_angle) {
  /* PID controller.
   * :param input_angle: angle from gyro.
   * :return: regulator product. */
  
  float error = regulator.setpoint - input_angle;
  regulator.integral = regulator.integral + error*dt;
  float derivative = (error - regulator.previous_error)/dt;
  regulator.previous_error = error;

  return (regulator.constants.Kp*error + regulator.constants.Ki*
      regulator.integral + regulator.constants.Kd*derivative);
}


void gyro_communication() {
  /* Read angle values from gyro via serial communication. */

  while (Serial3.available() && !communication.gyro.initialized) {
    if (Serial3.read() == '#') {
      communication.gyro.initialized = true;
    }
  }
  
  int i=0;
  if (Serial3.available()) {
    while (Serial3.available() && i < 10 && communication.gyro.initialized) {
      char input = Serial3.read();
      if (input == '#') {
        break;
      }
      communication.gyro.msg[i++] = input;
    }
    communication.gyro.gyro_angle = atof(communication.gyro.msg);
  }

  if (i > 0) {
    Serial.print(communication.gyro.gyro_angle);
    Serial.print(" ");
    Serial.println((char*)communication.gyro.msg);
  }
}
