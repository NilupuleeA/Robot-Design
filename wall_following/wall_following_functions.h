// Motor driver pins- ENTC
uint8_t INA_1=2;
uint8_t INB_1=23;
uint8_t INA_2=3;
uint8_t INB_2=22;

#define MAX_SPEED 150

int trigPin_bl = 53; // TRIG pin back left
int echoPin_bl = 51; // ECHO pin back left

int trigPin_br = 49; // TRIG pin back right
int echoPin_br = 47; // ECHO pin back right

int desiredDistance = 4; // Desired distance from the wall (ex: 4 cm)
int D1; // Distance from the left wall (when distance=0 or 50)
int D2; // Distance from the right wall (when distance=0 or 50)

int MotorBasespeed = 40;
int offset = 7;  // Temporary (If needed)

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error_bl = 0;
float error_br = 0;
float previousError = 0;

float wkp = 1;
float wkd = 0.1;
float wki = 0.0001;

void initializeMotors();
void initializeUltrasonicSensors();
void Rotate_Wheels(int speedA, int speedB);
int GetSensorReading(int trig, int echo);
void read_ultra();
void wPID_control();
void wpid_foward(int count);


/////////////////************///////////////////

// Function to initialize motors
void initializeMotors() {
  pinMode(INA_1, OUTPUT);
  pinMode(INB_1, OUTPUT);
  pinMode(INA_2, OUTPUT);
  pinMode(INB_2, OUTPUT); 
}

// Function to initialize ultrasonic sensors
void initializeUltrasonicSensors() {
  pinMode(trigPin_bl, OUTPUT);
  pinMode(echoPin_bl, INPUT);
  
  pinMode(trigPin_br, OUTPUT);
  pinMode(echoPin_br, INPUT);
}

// Function to induce a shock by rotating the wheels briefly
void shock() {
    Rotate_Wheels(100, 100);
    delay(40);
    Rotate_Wheels(0, 0);
}

// Function to get distance from ultrasonic sensors
int GetSensorReading(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  int duration = pulseIn(echo, HIGH);
  int distance = duration / 29 / 2;
  return distance;
}

// Function to keep the limits for distance from the left wall
int read_ultra(int trig, int echo) {
  int distance = GetSensorReading(trig, echo);
  if (distance>50 || distance==0){
    distance = 50;
  }
  return distance;
}

void wPID_control() {
  D1 = read_ultra(trigPin_bl, echoPin_bl);
  Serial.print("D1: ");
  Serial.print(D1);
  Serial.print(" ");

  D2 = read_ultra(trigPin_br, echoPin_br);
  Serial.print("D2: ");
  Serial.print(D2);
  Serial.print(" ");

  if(D1<20){
    error_bl = D1 - desiredDistance;   // Calculate error 
    P = error_bl; 
    I = I + error_bl;   // Update error sum for integral term
    D = error_bl - previousError;   // Calculate derivative term
    previousError = error_bl;   // Set the current error for next iteration

    speedAdjust = (wkp * P + wki * I + wkd * D);
    LMotorSpeed = MotorBasespeed - speedAdjust + offset;
    RMotorSpeed = MotorBasespeed + speedAdjust - offset;
  }

  if(D2<20){
    error_br = D2 - desiredDistance;   // Calculate error  
    P = error_br; 
    I = I + error_br;   // Update error sum for integral term
    D = error_br - previousError;   // Calculate derivative term
    previousError = error_br;   // Set the current error for next iteration

    speedAdjust = (wkp * P + wki * I + wkd * D);
    LMotorSpeed = MotorBasespeed + speedAdjust + offset;
    RMotorSpeed = MotorBasespeed - speedAdjust - offset;
  }
  
  else{
    Rotate_Wheels(LMotorSpeed,RMotorSpeed+12); 
  }    

  // Print the motor speed
  Serial.print(" speedAdjust ");
  Serial.print(speedAdjust);
  Serial.print(" "); 

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

  Rotate_Wheels(LMotorSpeed,RMotorSpeed+12);

  Serial.print("LMS: ");
  Serial.print(LMotorSpeed);
  Serial.print(" RMS: ");
  Serial.print(RMotorSpeed);
  Serial.print(" ");
  Serial.println();

}

// Function to move forward the robot according to PID signals
void wpid_foward() {
  read_ultra(trigPin_bl, echoPin_bl);
  read_ultra(trigPin_br, echoPin_br);
  wPID_control();
  Rotate_Wheels(LMotorSpeed, RMotorSpeed);
  
  delay(100);
}

// Function to rotate wheels based on speeds
void Rotate_Wheels(int speedA, int speedB) {
  if (speedA > 0) {
    digitalWrite(INB_1, LOW);
    analogWrite(INA_1, speedA);
  } else {
    digitalWrite(INB_1, HIGH);
    analogWrite(INA_1, -speedA);
  }
  if (speedB > 0) {
    digitalWrite(INB_2, LOW);
    analogWrite(INA_2, speedB);
  } else {
    digitalWrite(INB_2, HIGH);
    analogWrite(INA_2, -speedB);
  }
}