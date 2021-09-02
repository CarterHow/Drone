/*
The Sketch must be uploaded through the ICSP with an adapter as both D0 and D1 are being used. 
INSERT LINK
MPU Pins:
3V3 - 3.3V
SDA - A4
SCL - A5
GND - GND
INT - N/A
ESC Pins:
M1 - D4
M2 - D5
M3 - D6
M4 - D7
RF Receiver Pins:
C1 - D8
C2 - D9
C3 - D10
C4 - D11
5V - 5V
GND - GND
RGB LED Pins:
R - D2
G - D3
B - D12
Ultra Sonic Sensor 
5V - 5V
ECHO - D1
TRIG - D0
GND - GND
Buzzer
+ - D13
GND - GND
Quadcopter orientation (see Github page for diagrams).
Channel1(Roll), Channel2(Pitch), Channel3(Throttle), Channel4(Yaw)
Left Joystick(Throttle, Yaw), Right Joystick(Pitch, Roll)
Motor1(Front Left-Clockwise), Motor2(Front Right-Counter Clockwise), Motor3(Rear Left-Clockwise), Motor4(Rear Left-Counter Clockwise)

*/
#include <Arduino.h>
#include <Wire.h>
//----------Defining constants----------
//-----Radio-----
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
//-----Controls-----
#define YAW 0
#define PITCH 1
#define ROLL 2
#define THROTTLE 3
//-----MPU-----
#define X 0 //X Axis
#define Y 1 //Y Axis
#define Z 2 //Z Axis
#define FREQ 250 //Sampling Frequency
#define SSF_GYRO 65.5 //Sensitivity Scale Factor of the Gyro from the datasheet
//-----Flight-----
#define STOPPED 0
#define STARTING 1
#define STARTED 2
//----------Receiver Variables----------
//Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];
//Duration of the pulse on each channel of the receiver in us (musty be within 1000-2000us)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};
//Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4];//Timer of each channel
//Used to configure which control(Yaw, Pitch, Roll, Throttle) is on which channel
int mode_mapping[4];
//----------MPU Variables----------
//All in the order of x, y, z
//The Raw values produced from the gyro (in deg/sec)
int gyro_raw[3] = {0, 0, 0};
//Average gyro offsets of each axis
long gyro_offset[3] = {0, 0, 0};
//Calculated angles from the gyro values 
float gyro_angle[3] = {0, 0, 0};
//The Raw values from the Accelerometer (in m/sec^2)
int acc_raw[3] = {0, 0, 0};
//Calculated angles from the Accelerometer values
float acc_angle[3] = {0, 0, 0};
//Total 3D Accelerometer vector in m/s^2
long acc_total_vector;
//Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};
/*
Measures 3 Axis calculated from the gyro and accelerometer in order of: Yaw, Pitch, Roll
-Left wing up indicates a positive Roll
-Nose up indicates a positive Pitch
-Nose right indicates a positive Yaw
*/
float measures[3] = {0, 0, 0};
//-----MPU Temperature-----
int temperature;
//Init flag set to TRUE after the first loop
boolean initialized; 
//----------Servo Signal Generation Variables----------
unsigned int period;//Sampling period
unsigned long loop_timer;
unsigned long now, difference;
unsigned long pulse_length_esc1 = 1000, pulse_length_esc2 = 1000, pulse_length_esc3 = 1000, pulse_length_esc4 = 1000;
//----------PID Global Variables----------
//All in order of Yaw, Pitch, Roll
float pid_set_points[3] = {0, 0, 0};
//-----Errors-----
float errors[3];//Measured errors(compared to instructions)
float delta_err[3] = {0, 0, 0};//Error deltas
float error_sum[3] = {0, 0, 0};//Error sums(used for integral component)
float previous_error[3] = {0, 0, 0};//Last errors(used for derivative component)
//-----PID Coessicients-----
float Kp[3] = {4.0, 1.3, 1.3};//P coefficients
float Ki[3] = {0.02, 0.04, 0.04};//I coefficients
float Kd[3] = {0, 18, 18};//D coefficients
/*
Startus of the drone:
0:Stopped
1:Starting
2:Started
*/
//-----RGB Status LED-----
const int R = 2;
const int G = 3;
const int B = 12;
//-----UltraSonic Sensor-----
const int trigPin = 0;//Trigger Pin 
const int echoPin = 1;//Echo Pin 
bool Grounded;
//-----Buzzer-----
const int buzzer = 13;

int status = STOPPED;
int battery_voltage;

long microsecToCM(long microseconds)
{
    return microseconds / 29 / 2;
}
void isGrounded()//Used at setup to make sure the quadcopter is grounded to make sure that nothing else goes ahaid that could ruin callibrations
{
    long duration, cm;
    pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    cm = microsecToCM(duration);
    delay(100);
    if(cm > 5)
    {
        Grounded = false;
    }
    else if(cm < 5)
    {
        Grounded = true;
    }
}

void setupMPU()//Configure Gyro and Accelerometer precision
{
    //Configure Power Management
    Wire.beginTransmission(0x68);//Start communication woth MPU
    Wire.write(0x6B);//Request the PWR_MGMT_1 register
    Wire.write(0x00);//Apply the desired configuration to the register
    Wire.endTransmission(true);//End trnasmission
    //Configure Gyro Sensitivity
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);//Request the GYRO_CONFIG register
    Wire.write(0x10);//Set register to 1000deg/sec
    Wire.endTransmission(true);
    //Configure Accelerometer Sensitivity
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);//Request the ACCEL_CONFIG register
    Wire.write(0x10);//Set register to +-8g
    Wire.endTransmission(true);
    //Configure Low Pass Filter
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);//Request the CONFIG register
    Wire.write(0x03);//Set Digital Low Pass Filter (about 43Hz)
    Wire.endTransmission(true);
}

void readSensor()//Request the raw values from the MPU
{
    Wire.beginTransmission(0x68);//Start communication with MPU-6050
    Wire.write(0x3B);//Send the requested starting register 
    Wire.endTransmission(true);//End transmission
    Wire.requestFrom(0x68, 14);//Request 14 bytes from MPU
    //Wait untill all bytes are received 
    while(Wire.available() < 14);
    //Adding low and high bytes to the variable 
    acc_raw[X] = Wire.read() << 8 | Wire.read();
    acc_raw[Y] = Wire.read() << 8 | Wire.read();
    acc_raw[Z] = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_raw[X] = Wire.read() << 8 | Wire.read();
    gyro_raw[Y] = Wire.read() << 8 | Wire.read();
    gyro_raw[Z] = Wire.read() << 8 | Wire.read();
}

void calibrateMPU()//Calibrate MPU by taking 2000 samples and calculating the average offsets
{
    //During this step, the quadcopter needs to be static and on a level surface
    for(int i = 0; i < 2000; i++)
    {
        readSensor();
        //Assigning variables
        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];
        //generate low throttle pulse to initialize ESC 
        PORTD |= B11110000;//Set pins D4, 5, 6, 7 HIGH
        delayMicroseconds(1000);//Wait 1000us 
        PORTD &= B00001111;//Then set LOW
        //Short delay
        delay(3);
    }
    //Calculate average offsets
    gyro_offset[X] /= 2000;
    gyro_offset[Y] /= 2000;
    gyro_offset[Z] /= 2000;
}

void configureChannelMapping()
{
    mode_mapping[YAW] = CHANNEL4;
    mode_mapping[PITCH] = CHANNEL2;
    mode_mapping[ROLL] = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

void RGB_colour(int R_value, int G_value, int B_value)
{
    analogWrite(R, R_value);
    analogWrite(G, G_value);
    analogWrite(B, B_value);
}

void setup()
{
    //Start I2C communication
    Wire.begin();
    TWBR = 12;//Start I2C clock speed to 400kHz
    //Setting the RGB LED Pins
    pinMode(R, OUTPUT);
    pinMode(G, OUTPUT);
    pinMode(B, OUTPUT);
    RGB_colour(255, 255, 255);//WHITE
    //Set pins D4, 5, 6, 7 as Outpu
    DDRD |= B11110000;
    //MPU
    setupMPU();
    calibrateMPU();
    //RF
    configureChannelMapping();
    //Configure Interrupts for receiving
    PCICR |= (1 << PCIE0);//Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);//Set PCINT0 (Digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);//Set PCINT1 (Digital input 9) to trigger an unterrupt on state chnage.
    PCMSK0 |= (1 << PCINT2);//Set PCINT2 (Digital input 10) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);//Set PCINT3 (Digital input 11) to trigger an interrupt on state change.
    //Sampling Rate
    period = (1000000/FREQ);//Sampling period in us
    //Initialize loop_timer 
    loop_timer = micros();
    //Turn LED Blue 
    RGB_colour(0, 0, 255);
}

void stopAll()//Reset all motors' pulse length to 1000us to totally stop them.
{
    pulse_length_esc1 = 1000;
    pulse_length_esc2 = 1000;
    pulse_length_esc3 = 1000;
    pulse_length_esc4 = 1000;
}

void resetGyroAngles()//Reset Gyro angles with Accelerometer angles
{
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

void resetPidController()//Reset all PID controller's variables
{
    errors[YAW] = 0;
    errors[PITCH] = 0;
    errors[ROLL] = 0;

    error_sum[YAW] = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL] = 0;

    previous_error[YAW] = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL] = 0;
}

bool isStarted()//Return whether the quadcopter has started. 
{
    //To start the quadcopter, move the left stick to bottom left corner, then move back to center position.
    //To stop the quadcopter move the left stick in bottom right corner.
    if(status == STOPPED && pulse_length[mode_mapping[YAW]] <= 1012 && pulse_length[mode_mapping[THROTTLE]] <= 1012)
    {
        status = STARTING;
    }
    //When left stick is moved back in the center position 
    if(status == STARTING && pulse_length[mode_mapping[YAW]] == 1500 && pulse_length[mode_mapping[THROTTLE]] <= 1012)
    {
        status = STARTED;
        //Reset the PID controller's variables to prevent bump start 
        resetPidController();
        resetGyroAngles();
    }
    //When left stick is moved in the bottom right corner
    if(status == STARTED && pulse_length[mode_mapping[YAW]] >= 1988 && pulse_length[mode_mapping[THROTTLE]] <= 1012)
    {
        status = STOPPED;
        //Make sure to always stop motors when status == STOPPED
        stopAll();
    }
    return status == STARTED;
}

void calculateGyroAngles()//Calculate Pitch and ROll angles using only the Gyro
{
    //Subtract the offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];
    //Calculate angle using integration 
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro_angle[Y] / (FREQ * SSF_GYRO));
    //Transfer Roll to Pitch if IMU has yawed 
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

void calculateAccAngles()//Calculate Pitch and Roll angles using only the Accelerometer 
{
    //Calculate total 3D Accelerometer vector 
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));
    //To prevent asin produce to NaN, make sure the input value is within [-1;+1]
    if(abs(acc_raw[X]) < acc_total_vector)
    {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI);//Asin gives the angle in radians. So need to convert it to degrees by doing (180/PI)
    }
    if(abs(acc_raw[Y]) < acc_total_vector)
    {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
    }
}

void calculateAngles()
{
    calculateGyroAngles();
    calculateAccAngles();
    if(initialized)
    {
        //Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    }
    else
    {
        //At very first start, initialze the gyro angles with accelerometer angles
        resetGyroAngles();
        initialized = true;
    }
    //To dampen the pitch and roll angles a complementary filter is used 
    measures[ROLL] = measures[ROLL] * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    measures[YAW] = -gyro_raw[Z] / SSF_GYRO;//Store the angular motion for this axis
    //Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL] = 0.7 * angular_motions[ROLL] + 0.3 * gyro_raw[X] / SSF_GYRO;
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
    angular_motions[YAW] = 0.7 * angular_motions[YAW] + 0.3 * gyro_raw[Z] / SSF_GYRO;
}

float minMax(float value, float min_value, float max_value)//Make sure that the given value is within the min and max values
{
    if(value > max_value)
    {
        value = max_value;
    }
    else if(value < min_value)
    {
        value = min_value;
    }
    return value;
}

float calculateSetPoint(float angle, int channel_pulse)//Calculate the PID set point in deg/sec
{
    float level_adjust = angle * 15;//Value 15 limits the maximum angle value to +-32.8 deg
    float set_point = 0;
    //Use a dead band of 16us for better results
    if(channel_pulse > 1500)
    {
        set_point = channel_pulse - 1508;
    }
    else if(channel_pulse < 1492)
    {
        set_point = channel_pulse - 1492;
    }
    set_point -= level_adjust;
    set_point /= 3;
    return set_point;
}

float calculateYawSetPoint(int yaw_pulse, int throttle_pulse)//Calculate the PID set point of YAW axis in deg/s
{
    float set_point = 0;
    //Do not yaw when turning off the motors
    if(throttle_pulse > 1050)
    {
        //There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }
    return set_point;
}

void calculateSetPoints()//Calculate PID set points on axis YAW, PITCH, ROLL
{
    pid_set_points[YAW] = calculateYawSetPoint(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[mode_mapping[PITCH]]);
    pid_set_points[ROLL] = calculateSetPoint(measures[ROLL], pulse_length[mode_mapping[ROLL]]);
}

void calculateErrors()//Calculate errors used by PID controller
{
    //Calculate current errors
    errors[YAW] = angular_motions[YAW] - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL] = angular_motions[ROLL] - pid_set_points[ROLL];
    //Calculate sum of errors (Integral coefficient)
    error_sum[YAW] += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL] += errors[ROLL];
    ////Keep values within the acceptable range 
    error_sum[YAW] = minMax(error_sum[YAW], -400/Ki[YAW], 400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL] = minMax(error_sum[ROLL], -400/Ki[ROLL], 400/Ki[ROLL]);
    //Calculate error delta (Derivative coefficients)
    delta_err[YAW] = errors[YAW] - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL] = errors[ROLL] - previous_error[ROLL];
    //Save the current error as previouse_error for next time 
    previous_error[YAW] = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL] = errors[ROLL];
}

void pidController()//
{
    float yaw_pid, pitch_pid, roll_pid = 0;
    int throttle = pulse_length[mode_mapping[THROTTLE]];
    //Initialize motor commands with throttle 
    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;
    //Do not calculate anything if throttle is 0
    if(throttle >= 1012)
    {
        //PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid = (errors[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (delta_err[YAW] * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid = (errors[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (delta_err[ROLL] * Kd[ROLL]);
        //Keep values within range 
        yaw_pid = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid = minMax(roll_pid, -400, 400);
        //Calculate pulse duration for each ESC
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
    }
    //Prevent out-of-range values
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

bool isBatteryConnected()
{
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
    return battery_voltage < 1240 && battery_voltage > 800;
}

void compensateBatteryDrop()
{
    if(isBatteryConnected())
    {
        pulse_length_esc1 += pulse_length_esc1 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc2 += pulse_length_esc2 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc3 += pulse_length_esc3 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc4 += pulse_length_esc4 * ((1240 - battery_voltage) / (float) 3500);
    }
}

void applyMotorSpeed()
{
    //Refresh rate is 250Hz: send ESC pulses every 4000us
    while((now = micros()) - loop_timer < period)
    {
        //Update the loop timer
        loop_timer = now;
        //Set pins D4, 5, 6, 7 as LOW
        PORTD |= B11110000;
        //Wait untill all pins are LOW
        while(PORTD >= 16)
        {
            now = micros();
            if(difference >= pulse_length_esc1) PORTD &= B11101111;//Set pin D4 LOW
            if(difference >= pulse_length_esc2) PORTD &= B11011111;//Set pin D5 LOW
            if(difference >= pulse_length_esc3) PORTD &= B10111111;//Set pin D6 LOW
            if(difference >= pulse_length_esc4) PORTD &= B01111111;//Set pin D7 LOW
        }
    }
}

void loop()
{
    //First read the raw values from the MPU
    readSensor();
    //Calculate angles from the Gyro and Accelerometer values
    calculateAngles();
    //Calculate set points of PID controller
    calculateSetPoints();
    //Calculate errors comparing angular motions to set points
    calculateErrors();
    //Mode sellect options
    if(isStarted())
    {
        pidController();

        compensateBatteryDrop();
    }
    applyMotorSpeed();
}
/*
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 *
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
 * It is less convenient but more efficient, which is the most important here.
 *
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 * @see https://www.firediy.fr/article/utiliser-sa-radiocommande-avec-un-arduino-drone-ch-6
 */
ISR(PCINT0_vect) 
{
    current_time = micros();
    // Channel 1 -------------------------------------------------
    if (PINB & B00000001)//Is input 8 high ? 
    {                                      
        if (previous_state[CHANNEL1] == LOW)//Input 8 changed from 0 to 1 (rising edge) 
        {                     
            previous_state[CHANNEL1] = HIGH;//Save current state
            timer[CHANNEL1] = current_time;//Save current time
        }
    } 
    else if (previous_state[CHANNEL1] == HIGH)//Input 8 changed from 1 to 0 (falling edge)
    {                
        previous_state[CHANNEL1] = LOW;//Save current state
        pulse_length[CHANNEL1] = current_time - timer[CHANNEL1];//Calculate pulse duration & save it
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00000010)//Is input 9 high ?
    {                                       
        if (previous_state[CHANNEL2] == LOW)//Input 9 changed from 0 to 1 (rising edge)
        {                    
            previous_state[CHANNEL2] = HIGH;//Save current state
            timer[CHANNEL2] = current_time;//Save current time
        }
    }
    else if (previous_state[CHANNEL2] == HIGH)//Input 9 changed from 1 to 0 (falling edge)
    {                
        previous_state[CHANNEL2] = LOW;//Save current state
        pulse_length[CHANNEL2] = current_time - timer[CHANNEL2];//Calculate pulse duration & save it
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100)//Is input 10 high ?
    {                                       
        if (previous_state[CHANNEL3] == LOW)//Input 10 changed from 0 to 1 (rising edge)
        {                     
            previous_state[CHANNEL3] = HIGH;//Save current state
            timer[CHANNEL3] = current_time;//Save current time
        }
    } 
    else if (previous_state[CHANNEL3] == HIGH)//Input 10 changed from 1 to 0 (falling edge)
    {                
        previous_state[CHANNEL3] = LOW;//Save current state
        pulse_length[CHANNEL3] = current_time - timer[CHANNEL3];//Calculate pulse duration & save it
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B00001000)//Is input 11 high ?
    {                                       
        if (previous_state[CHANNEL4] == LOW)//Input 11 changed from 0 to 1 (rising edge)
        {                    
            previous_state[CHANNEL4] = HIGH;//Save current state
            timer[CHANNEL4] = current_time;//Save current time
        }
    } 
        else if (previous_state[CHANNEL4] == HIGH)//Input 11 changed from 1 to 0 (falling edge)
    {                 
        previous_state[CHANNEL4] = LOW;//Save current state
        pulse_length[CHANNEL4] = current_time - timer[CHANNEL4];//Calculate pulse duration & save it
    }
}
