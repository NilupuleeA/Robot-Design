uint16_t threshold = 150; // Threshold value for IR sensors
uint16_t sensorValues[8];
uint8_t IR_digital_array[8];
uint8_t IR_pins[] = {A8, A9, A10, A11, A12, A13, A14, A15}; // IR array pins

// Motor driver pins
// INA - Speed control pins
// INB - Phase control pins
uint8_t INA_1 = 2;
uint8_t INB_1 = 23;
uint8_t INA_2 = 3;
uint8_t INB_2 = 22;

uint16_t Basespeed = 50; 
int control_signal_1; 
int control_signal_2; 

// PID values and errors
float kp = 0.0044;
float kd = 0.022; //0.018
float ki = 0.0007;

int error; 
int previouserror = 0;
int total_error = 0; 

int integral = 0;
int count = 0;

// Function to initialize IR sensors
void initializeIRSensors() {
    // Configure IR array pins
    for (uint8_t i = 0; i < 8; i++) {
        pinMode(IR_pins[i], INPUT);
    }

    // Start indicator
    for (uint8_t i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
}

// Function to initialize motors
void initializeMotors() {
    // Motor Driver Pins
    pinMode(INA_1, OUTPUT);
    pinMode(INB_1, OUTPUT);
    pinMode(INA_2, OUTPUT);
    pinMode(INB_2, OUTPUT);
}

/////////////////************///////////////////

// Function to rotate wheels based on speeds
void Rotate_Wheels(int speedA, int speedB) {
    // speedA = constrain(speedA, -150, 150);
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

// Function to induce a shock by rotating the wheels briefly
void shock() {
    Rotate_Wheels(100, 100);
    delay(40);
    Rotate_Wheels(0, 0);
}

// Function to perform digital read on a pin with a threshold
uint8_t digital_read(uint8_t pin) {
    int IR_analog_value = analogRead(pin);
    if (IR_analog_value < threshold) {
        return 1;
    } else {
        return 0;
    }
}

// Function to calculate the error based on digital readings of IR sensors
int error_v() {
    total_error = 0;
    count = 0;
    int error_v; 
    for (uint8_t i = 0; i < 8; i++) {
        IR_digital_array[i] = digital_read(IR_pins[i]); 
        if (IR_digital_array[i] == 1) {
            count += 1;
            total_error = total_error + 1000 * i;
            sensorValues[i] = analogRead(IR_pins[i]); 
        }

        sensorValues[i] = analogRead(IR_pins[i]);
    }

    if (count != 0) {
        error_v = total_error / count - 3500;
    } else {
        // Handle the case when count is 0 if needed
    }
    return error_v;
}

// Function to perform PID line following
void pid_line_following() {
    int currenterror = error_v();
    error = currenterror;
    integral += currenterror; 
    int delta = currenterror - previouserror;
    float CT = currenterror * kp + delta * kd + integral * ki;
    previouserror = currenterror; 
    control_signal_2 = Basespeed + round(constrain(CT, -255, 255)); 
    control_signal_1 = Basespeed - round(constrain(CT, -255, 255));

    // motor_control(1, control_signal_1); motor_control(2, control_signal_2);
    Rotate_Wheels(control_signal_1, control_signal_2);
    delay(4);
}

// Function for debugging PID parameters and sensor values
void PID_debug() {
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println('\t');
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print(IR_digital_array[i]);
        Serial.print('\t');
    }
    Serial.print(control_signal_1);
    Serial.print('\t');
    Serial.print(control_signal_2);
    Serial.print('\t');
    Serial.print(error);
    Serial.print('\t');
    Serial.print(count);
    Serial.print('\t');
    Serial.println(total_error);
}
