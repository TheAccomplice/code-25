#ifndef PID_H
#define PID_H

class PID
{
    public:
        // setpoint - desired value
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable

        PID(double setpoint, double min, double max, double kp, double ki, double kd, double dt);   
        void updateSetpoint(double setpoint);
        void updateGains(double kp, double ki, double kd);
        double getSetpoint();
        void updateLimits(double min, double max);

        void reset();

        
        double advance(double processvariable);

        private:
            double _setpoint;
            double _min;
            double _max;
            double _kp;
            double _ki;
            double _kd;
            double _dt;
            // internal values
            double _integral;
            double _error;
            double _lasterror;
            double _lastoutput;
            double _lasttime;
};
#endif