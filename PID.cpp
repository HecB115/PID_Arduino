  /*PID.cpp - Library for applying PID controllers.
  Created by Héctor Brizuela, December 17, 2022.
  Released into the public domain.
  
  GIT: https://github.com/HecB115
  */  

#include "PID.h"

PID::PID(double Kp, double Ki, double Kd){
  // We set our Kp, Ki, and Kd values
  kp = Kp;
  ki = Ki;
  kd = Kd;

  /* The default Limits of the PID are 0 - 255 but you can change it to whatever values you like
  in the PID::limits function */
  minVal = 0;
  maxVal = 255;

  //The default time samples are 50 ms but you can change it in the PID::time function
  TimeSamples = 50;
}

double PID::time(double timesamples){
  TimeSamples = timesamples;
  return TimeSamples;
}

double PID::limits(double minValue, double maxValue){
  minVal = minValue;
  maxVal = maxValue;

  return minVal, maxVal;
}

// If you want to change the current Kp, Ki and Kd values, use this function
double PID::kValues(double KP, double KI, double KD){
  kp = KP;
  ki = KI;
  kd = KD;  
}

//Inside this function happens all the magic
double PID::output(double Input, double DesiredValue){
  currentTime = millis();
  double dt = currentTime - lastTime;

/* This function should be placed in your void loop and it wil be called every 50 ms or every
TimeSample you've chosed */

  if (dt > TimeSamples) 
  { 
    currentError = DesiredValue - Input; //The Current Error is defined as the difference between the desired input value and the current value Input
    cumError += currentError; //We calculate the Cumulated Error
    rateOfError = currentError - lastError; // And we calculate the rate of change in the error

    // Wee obtain the p, i , and d values of our controller to obtain the PID value
    p = kp * currentError;
    i = ki * cumError;
    d = kd * rateOfError;

    //We only use the integral value when we are close to the desired Value to avoid a possible integral windup
    if (3 > currentError && -3 < currentError) i *= 1; 
    else i = 0;

    pid = p + i + d;

    //We enclose the PID values to the max and min limits we established
    if (pid > maxVal) pid = maxVal;
    if (pid < minVal) pid = minVal;
    
    //Update of parameters
    lastTime = currentTime;
    lastError = currentError;

    return pid; //Result
  }
}