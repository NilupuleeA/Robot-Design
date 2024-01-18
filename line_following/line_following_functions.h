#define RMotorA 3   // Right motor forward direction
#define RMotorB 4   // Right motor backward direction
#define RMotorPWM 6 // Right motor PWM

#define LMotorA 2   // Left motor forward direction
#define LMotorB 7   // Left motor backward direction
#define LMotorPWM 5 // Left motor PWM

#define MAX_SPEED 100

// PID constants
float kp = 0.066; 
float kd = 0.1;
float ki = 0.0001;

float P, I, D;
float currenterror = 0;
float previousError = 0;

int previouserror;

int MotorBasespeed = 80;
int offset = 7;

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

// Pin definitions for IR sensors
int IR_sensor0_pin = 0; 
int IR_sensor1_pin = 1; 
int IR_sensor2_pin = 2; 
int IR_sensor3_pin = 3; 
int IR_sensor4_pin = 4; 
int IR_sensor5_pin = 5; 
int IR_sensor6_pin = 6; 
int IR_sensor7_pin = 7;

int IR0, IR1, IR2, IR3, IR4, IR5, IR6, IR7;
int threshold = 500;

void initializeMotors();
void initializeIRSensors();
void shock();
void set_speed();
void set_forward();
int digital_read(int pin);
int error_v();
void stop();
void PID_control();

//----------------------------------------------------

// Function to initialize motors
void initializeMotors() {
  // Initialize Left Motor
  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  // Initialize Right Motor
  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);
}

// Function to initialize IR sensors
void initializeIRSensors() {
  pinMode(IR_sensor0_pin,INPUT);
  pinMode(IR_sensor1_pin,INPUT);
  pinMode(IR_sensor2_pin,INPUT);
  pinMode(IR_sensor3_pin,INPUT);
  pinMode(IR_sensor4_pin,INPUT);
  pinMode(IR_sensor5_pin,INPUT);
  pinMode(IR_sensor6_pin,INPUT);
  pinMode(IR_sensor7_pin,INPUT);
}

// Function to give shock to motors (forward direction)
void shock() {
  set_forward();
  analogWrite(LMotorPWM, 255);
  analogWrite(RMotorPWM, 255);
  delay(10);
  analogWrite(LMotorPWM, 0);
  analogWrite(RMotorPWM, 0);
}

// Function to adjust speed of motors according to the PID signals
void set_speed() {
  analogWrite(LMotorPWM, LMotorSpeed);
  analogWrite(RMotorPWM, RMotorSpeed);
}

// Enables motors to move forward
void set_forward() {
  digitalWrite(LMotorA, HIGH);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorB, LOW);
}

// Function to convert analog inputs into digital inputs
int digital_read(int pin)
{
  int IR_analog_value=analogRead(pin);

  // Compare the analog anput and the threshold value
  if (IR_analog_value<threshold)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

// Funtion to calculate error
int error_v()
{ 
  IR0=digital_read(IR_sensor0_pin);
  Serial.print(IR0);
  IR1=digital_read(IR_sensor1_pin);
  Serial.print(IR1);
  IR2=digital_read(IR_sensor2_pin);
  Serial.print(IR2);
  IR3=digital_read(IR_sensor3_pin);
  Serial.print(IR3);
  IR4=digital_read(IR_sensor4_pin);
  Serial.print(IR4);     
  IR5=digital_read(IR_sensor5_pin);
  Serial.print(IR5);
  IR6=digital_read(IR_sensor6_pin);
  Serial.print(IR6);
  IR7=digital_read(IR_sensor7_pin); 
  Serial.print(IR7);  
  Serial.print("  ");   

  // Multiply each digital input by its allocated weight
  int error_v = (IR0 * 800 + IR1 * 500 + IR2 * 200 + IR3 * 0 + IR4 * 0 + IR5 * -200 + IR6 * -500 + IR7 * -800) ;
  return error_v;
}

// Enables stop command
void stop() {
  digitalWrite(LMotorA, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorB, LOW);
}

// Function to apply PID for line following
void PID_control() {
  // Calculate error
  int currenterror = error_v(); 
  P = currenterror;

  // Update error sum for integral term
  I = I + currenterror;

  // Calculate derivative term
  D = currenterror - previousError;

  // Set the current error for next iteration
  previousError = currenterror;

  // Calculate PID output
  speedAdjust = (kp * P + ki * I + kd * D);

  // Print the motor speed
  Serial.print(" speedAdjust ");
  Serial.print(speedAdjust);
  Serial.print(" ");

  // Set motor speeds
  LMotorSpeed = MotorBasespeed + speedAdjust;
  RMotorSpeed = MotorBasespeed - speedAdjust;

  // Keep the speeds in the limits
  if (LMotorSpeed < 0)
  {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0)
  {
    RMotorSpeed = 0;
  }
  if (LMotorSpeed > MAX_SPEED)
  {
    LMotorSpeed = MAX_SPEED;
  }
  if (RMotorSpeed > MAX_SPEED)
  {
    RMotorSpeed = MAX_SPEED;
  }

  Serial.print("LMS: ");
  Serial.print(LMotorSpeed);
  Serial.print(" RMS: ");
  Serial.print(RMotorSpeed);
  Serial.print(" ");
  Serial.println();
}

// Function to move the robot according to PID signals
void pid_line_following() {
  shock();
  while (!(((digital_read(IR_sensor0_pin)==1)&&(digital_read(IR_sensor1_pin)==1)&&(digital_read(IR_sensor2_pin)==1)&&(digital_read(IR_sensor3_pin)==1)&&(digital_read(IR_sensor4_pin)==1)&&(digital_read(IR_sensor5_pin)==1)&&(digital_read(IR_sensor6_pin)==1)&&(digital_read(IR_sensor7_pin)==1))||((digital_read(IR_sensor0_pin)==0)&&(digital_read(IR_sensor1_pin)==0)&&(digital_read(IR_sensor4_pin)==1)&&(digital_read(IR_sensor5_pin)==1)&&(digital_read(IR_sensor6_pin)==1)&&(digital_read(IR_sensor7_pin)==1))||((digital_read(IR_sensor0_pin)==1)&&(digital_read(IR_sensor1_pin)==1)&&(digital_read(IR_sensor2_pin)==1)&&(digital_read(IR_sensor3_pin)==1)&&(digital_read(IR_sensor6_pin)==0)&&(digital_read(IR_sensor7_pin)==0))))
  {
    PID_control();
  }
  stop();
}

