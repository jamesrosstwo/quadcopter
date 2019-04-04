#include <Servo.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#include <MPU6050.h>
#include <I2Cdev.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte highByte, lowByte;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, gyro_address;
int receiver_input[5];
int16_t acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output 
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;

Servo ESCs[4];

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define HORIZ_TILT_CH  0
#define VERT_TILT_CH  1
#define THROTTLE_CH  2
#define YAW_CH  3

#define HORIZ_TILT_CH_INPUT  A0
#define VERT_TILT_CH_INPUT  A1
#define THROTTLE_CH_INPUT  A2
#define YAW_CH_INPUT  A3

// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-6050 Register Map and Descriptions, Revision 4.0, RM-MPU-6050A-00, 9/12/2012 for registers not listed in 
// above document; the MPU6050 and MPU-9150 are virtually identical but the latter has an on-board magnetic sensor
//..
#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68  // Device address when ADO = 0
#endif

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
float aRes, gRes; // scale resolutions per LSB for the sensors

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(HORIZ_TILT_CH, HORIZ_TILT_CH_INPUT); }
void calc_ch2() { calc_input(VERT_TILT_CH, VERT_TILT_CH_INPUT); }
void calc_ch3() { calc_input(THROTTLE_CH, THROTTLE_CH_INPUT); }
void calc_ch4() { calc_input(YAW_CH, YAW_CH_INPUT); }

// spin each motor for 1 second each, then spin all motors for one second. stop all.
void testMotors(){
  for(int i = 0; i < 4; i++){
    ESCs[i].writeMicroseconds(1400);
    if(i > 0){
      ESCs[i-1].writeMicroseconds(1000); 
    }
    delay(1000);
  }
  for(int i = 0; i < 4; i++){
    ESCs[i].writeMicroseconds(1400);
  }
  delay(1000);
  for(int i = 0; i < 4; i++){
    ESCs[i].writeMicroseconds(1000);
  }
}

void gyro_signalen(){

  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt

    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // Now we'll calculate the accleration value into actual g's
    acc_axis[1] = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    acc_axis[2] = (float)accelCount[1]*aRes - accelBias[1];   
    acc_axis[3] = (float)accelCount[2]*aRes - accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gyro_axis[1] = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gyro_axis[2] = (float)gyroCount[1]*gRes - gyroBias[1];  
    gyro_axis[3] = (float)gyroCount[2]*gRes - gyroBias[2];   

    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
    Serial.println();
    Serial.println("working"); 
  }else{
    Serial.println();
    Serial.println("not working");  
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[1];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  gyro_pitch = gyro_axis[2];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  gyro_yaw = gyro_axis[3];

  acc_x = acc_axis[1];                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[2];
  acc_z = acc_axis[3];                           //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

void set_digital_ports(){
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
}

void get_battery_voltage(){
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();                                                    //Set the timer for the next loop.   
}

void setup_escs_and_receiver(){
  for(int i = 0; i < 4; i++){
    ESCs[i].attach(5+i);
    ESCs[i].writeMicroseconds(1000); //initialize the signal to 1000
  }
  Serial.println("ESCs attached");
  pinMode(HORIZ_TILT_CH_INPUT, INPUT);
  pinMode(VERT_TILT_CH_INPUT, INPUT);
  pinMode(THROTTLE_CH_INPUT, INPUT);
  pinMode(YAW_CH_INPUT, INPUT);
  Serial.println("Receiver channels initialized");

  attachInterrupt(HORIZ_TILT_CH_INPUT, calc_ch1, CHANGE);
  attachInterrupt(VERT_TILT_CH_INPUT, calc_ch2, CHANGE);
  attachInterrupt(THROTTLE_CH_INPUT, calc_ch3, CHANGE);
  attachInterrupt(YAW_CH_INPUT, calc_ch4, CHANGE);
  Serial.println("Interrupts attached");
}

void calculate_gyroscope_angles(){
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
}

void calculate_accelerometer_angles(){
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.

  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }
}
void calculate_pid_inputs(){
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(rc_values[HORIZ_TILT_CH] > 1508)pid_roll_setpoint = rc_values[HORIZ_TILT_CH] - 1508;
  else if(rc_values[HORIZ_TILT_CH] < 1492)pid_roll_setpoint = rc_values[HORIZ_TILT_CH] - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(rc_values[VERT_TILT_CH] > 1508)pid_pitch_setpoint = rc_values[VERT_TILT_CH] - 1508;
  else if(rc_values[VERT_TILT_CH] < 1492)pid_pitch_setpoint = rc_values[VERT_TILT_CH] - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(rc_values[THROTTLE_CH] > 1050){ //Do not yaw when turning off the motors.
    if(rc_values[YAW_CH] > 1508)pid_yaw_setpoint = (rc_values[YAW_CH] - 1508)/3.0;
    else if(rc_values[YAW_CH] < 1492)pid_yaw_setpoint = (rc_values[YAW_CH] - 1492)/3.0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void calculate_esc_pulses(){
  throttle = rc_values[THROTTLE_CH];                                      //We need the throttle signal as a base signal.

  if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
  esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

  if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
    esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
    esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
    esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
    esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
  }

  if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
  if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
  if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
  if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

  if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
  if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
  if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
  if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/fqEkVcqxtU8
  ////////////////////////////////////////////////////////////////////////////////////////////////////
}


void getGres() {
  switch (Gscale)
  {
   // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0]) << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initMPU6050()
{  
 // Initialize MPU6050 device

//  wake up device-don't need this here if using calibration function below
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
//  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

  // get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz sample rate 
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x02);    
   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00); 
  delay(200);
  
// Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000; 
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
}

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}


void start_MPU(){
  MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
  Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
  if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
    delay(1000);
  calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  delay(2000);
  Serial.begin(SERIAL_PORT_SPEED);
  pinMode(12, OUTPUT);
  //Use the led on the Arduino for startup indication.
  digitalWrite(12,HIGH);           
  //Turn on the warning led.
  Serial.println("working");
  setup_escs_and_receiver();
  Serial.println("ESCs and receiver setup");
  Wire.begin();
  start_MPU();
  Serial.println("MPU Initialized");
  Serial.println("Setup complete");
  digitalWrite(12, LOW);
  delay(2000);
}

void loop() {
  rc_read_values();

//  Serial.print("Horizontal Tilt:"); Serial.print(rc_values[HORIZ_TILT_CH]); Serial.print("\t");
//  Serial.print("Vertical Tilt:"); Serial.print(rc_values[VERT_TILT_CH]); Serial.print("\t");
//  Serial.print("Throttle Strength:"); Serial.print(rc_values[THROTTLE_CH]); Serial.print("\t");
//  Serial.print("Turn:"); Serial.println(rc_values[YAW_CH]); 
  calculate_gyroscope_angles();
  calculate_accelerometer_angles();
  manage_special_controls();
  calculate_pid_inputs();
  calculate_pid();

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);


  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !  
  //Set the timer for the next loop.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();
//  wait_for_low_outputs();
  calculate_esc_pulses();
  Serial.print("1:");
  Serial.print(esc_1);
  Serial.print(" 2:");
  Serial.print(esc_2);
  Serial.print(" 3:");
  Serial.print(esc_3);
  Serial.print(" 4:");
  Serial.print(esc_4);
  Serial.print(" | Gyro x:");
  Serial.print(gyro_axis[1]);
  Serial.print(" Gyro y:");
  Serial.print(gyro_axis[2]);
  Serial.print(" Gyro z:");
  Serial.print(gyro_axis[3]);
  
  Serial.println();
  ESCs[0].writeMicroseconds();
  ESCs[1].writeMicroseconds(esc_2);
  ESCs[2].writeMicroseconds(esc_3);
  ESCs[3].writeMicroseconds(esc_4);

  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();   
}
