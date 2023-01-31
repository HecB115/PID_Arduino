/*PID Example to control Aircraft Ailerons and Rudder with MPU6050 sensor
  to achieve desired roll and yaw values
 -Library for PID Controller implementation.
  Created by Héctor Brizuela, December 17, 2022.
  Released into the public domain.

  GIT: https://github.com/HecB115
*/

#include <Wire.h>
#include <Servo.h>
#include <PID.h>

#define Left_pin 11
#define Right_pin 10 
#define Back_pin 9

//We define the Kp, Ki and Kd values.
double Kp = 4, Ki = 0.01, Kd = 0.1;

//After that we set all the variables that will save the values that we want to know

//Raw values of accelerometer and gyroscope
int16_t Ax_RAW, Ay_RAW, Az_RAW;
int16_t Gx_RAW, Gy_RAW, Gz_RAW;

//Timers
unsigned long currentTime, previousTime; 
double changeInTime;

//Accelerations in g's in all three axis
float ax, ay, az;

//Angular velocity in °/s for X and Y axis
float gx, gy;

//Angular displacement in X and Y axis obtained by Euler angles
float pitch, roll;

//Angular displacement obtained by applying a complementary filter
double realGx, realGy;

//PID output
double OutputRoll, OutputYaw;

//Call and set the Servo and PID libraries
Servo Left, Right, Back;
PID pid(Kp, Ki, Kd);

/*Note: Important to remember that the ACCEL_XOUT_H register is 3B,
the GYRO_XOUT_H register Addres is 43 and the I2C adress of the MPU6050
is 68
Source: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
*/


void setup(){
  Serial.begin(115200); //Initialze Serial Communication
  Wire.begin(); //Initialize I2C communication
  Wire.beginTransmission(0x68); //MPU6050 I2C Address is 0x68 by default
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); //Place a 0 into the 6B register
  Wire.endTransmission(true); //End Transmission

  // Indicate the servo digital pin connections
  Left.attach(Left_pin); 
  Right.attach(Right_pin);
  Back.attach(Back_pin);

  pid.time(50); //Set the time samples
  pid.limits(-90 , 90);  //Set the minimal and maximal values of the PID
}

float rad2degs(float val){
  //Function to turn radians to degrees
  val *= 180 / M_PI;
  return val;
}
void readAcc(){
  //Function to read accelerometer values:
    
  Wire.beginTransmission(0x68); //Initialize transmission with MPU6050
  Wire.write(0x3B); //Get the ACCEL_XOUT_H register address
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  //Ask for the ACCEL_OUT_H and ACCEL_OUT_L for all three axis
  Ax_RAW = Wire.read() << 8 | Wire.read();
  Ay_RAW = Wire.read() << 8 | Wire.read();
  Az_RAW = Wire.read() << 8 | Wire.read();

  /*To convert raw accelerometer values to g's we need to divide the  
    raw values by 16384 */
  ax = Ax_RAW / 16384.0;
  ay = Ay_RAW / 16384.0;
  az = Az_RAW / 16384.0; 
  
  //Calculate the accelerometer part of complementary filter
  roll = rad2degs((atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)))));
  pitch = rad2degs((atan2(-ax , sqrt(pow(ay, 2) + pow(az, 2)))));
}

void readGyro(){
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //The Address of the GYRO_XOUT_H is 43
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true); 
  /*We only receive two 8 bits for each Gyroscope reading so we only need 4 because we aren't
    reading the Z axis value */

  Gx_RAW = Wire.read() << 8 | Wire.read();
  Gy_RAW = Wire.read() << 8 | Wire.read();

  //To turn the raw values of the gyroscope into °/s we need to divide by 131
  gx = Gx_RAW / 131.0;
  gy = Gy_RAW / 131.0;
}

//Function were we apply the complementary filter to the pitch and roll angles
void realAngles(){
  //Initialize Timers
  currentTime = micros();
  changeInTime = (currentTime - previousTime) / 1000000;

  //Get the accelerometer and gyroscope readings
  readAcc();
  readGyro();

  //Get filtered Angles
  realGx = 0.98 * (realGx + gx * changeInTime) + 0.02 * roll;
  realGy = 0.98 * (realGy + gy * changeInTime) + 0.02 * pitch;

  //Update timer
  previousTime = currentTime;
}

void loop(){
  realAngles(); //Get filtered angles
  /*Apply the PID controller by stating the PID input
  and the PID desired input to get the PID output */

  OutputRoll = pid.output(realGx, 0);
  OutputYaw = pid.output(realGy, 0);

  //The servo movements start in 90 and moves according to the PID output
  double LeftRotation = 90 + OutputRoll;
  double RightRotation = 90 - OutputRoll;
  double BackRotation = 90 + OutputYaw;

  //Move the Servos
  Left.write(LeftRotation);
  Right.write(RightRotation);
  Back.write(BackRotation);

 //And thats all :D hope this is helpful 
}


