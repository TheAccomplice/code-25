#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <pid.h>
#include <Arduino.h>

using namespace std;

// PID controller constructor
PID::PID(double setpoint, double min, double max, double kp, double ki, double kd, double dt)
{
    // note that "->this" is implicitly used here
    _setpoint = setpoint;
    _dt = dt;
    _max = max;
    _min = min;
    _kp = kp;
    _kd = kd;
    _ki = ki;
    _error = 0;
    _lasterror = 0;
    _integral = 0;
};

double PID::advance(double processvariable) {
    // Calculate error
    _error = _setpoint - processvariable;

    const auto now = micros();
    const auto dt = now - _lasttime;
    _lasttime = now;

    if (dt < _dt) return _lastoutput;

    double Pout = _kp * _error;

    _integral += _error * _dt;
    double Iout = _ki * _integral;

    double derivative = (_error - _lasterror) / _dt;
    double Dout = _kd * derivative;

    double output = Pout + Iout + Dout;

    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    _lasterror = _error;

    return output;
};

void PID::reset() {
    _error = 0;
    _lasterror = 0;
    _integral = 0;
    _lasttime = micros();
    _lastoutput = 0;
};

void PID::updateGains(double kp, double ki, double kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
};

//call before advance
void PID::updateSetpoint(double setpoint) {
    _setpoint = setpoint;
};

double PID::getSetpoint() {
    return _setpoint;
};

void PID::updateLimits(double min, double max) {
    _min = min;
    _max = max;
};  
#endif