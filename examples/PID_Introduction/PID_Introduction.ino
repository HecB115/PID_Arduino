/*Introductory example to the PID Library
  with the objetive of obtaining constant luminosity with a potentiometer
 -Library for PID Controller implementation.
  Created by Héctor Brizuela, December 17, 2022.
  Released into the public domain.

  GIT: https://github.com/HecB115
*/

#include <PID.h>
#define pwm_pin 11

//Proposed Kp, ki and Kd values, you can experiment to see wich values gives you the best results
double Kp = 10, Ki = 2.5, Kd = 0.001;

//Initialize the PID controller
PID pid(Kp, Ki, Kd);

void setup(){
/*PID time sampling is 50ms by default, if you want to change it
 use pid.time(Desired_Time_Sampling) */

 /* PID default limits are between 0 and 255 if you want to modify them
 you can call pid.limits(lower_limit, upper_limit) */

}

void loop(){
  double desiredvalue = 255; //We establish the desired Input value of the PID is 100% brightness
  double val = analogRead(A0); //The PID input is going to be the potentiometer readings
  val = map(val, 0, 1024, 0, 255); //We map the analog readings from [0, 255]
  double output = pid.output(val, desiredvalue); //Get PID value by introducing the PID current input and desired input
  analogWrite(pwm_pin, output); //Get constant luminosity while changing potentiometer positioning

 /* If at any point you want to change the Kp, Kd and Ki values
 just use the pid.kValues(newKp, newKi, newKd) function
 */
}
