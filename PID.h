  /*PID.h - Library for applying PID controllers.
  Created by Héctor Brizuela, Decenmber 17, 2022.
  Released into the public domain.

  GIT: https://github.com/HecB115
  */  

#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID{
  public:
  PID::PID(double Kp, double Ki, double Kd); //Begin the library and set Kp, kd and Ki values;
  double PID::time(double timesamples); //Default Sampling time is 50 ms but in you want to change it, use this fucntion
  double PID::limits(double minValue, double maxValue); /*The default limit of the PID value is between 0 and 255
  if you want to change them to whatever you want, use this fucntion */

  double PID::kValues(double KP, double KI, double KD); //If you want to change your current Kp , Ki and Kd values, use this fucntion
  double PID::output(double Input, double DesiredValue); //Function used to obtain the PID controller Value

  private:
  double kp, ki, kd, TimeSamples, maxVal, minVal, currentError, lastError, p, i, d, pid;
  double cumError, rateOfError, currentTime, lastTime;
};

#endif