#ifndef SHARED_H
#define SHARED_H

#include "vector.h"

struct TimeControl {
    double now = 0;
    double last = 0;
    double dt = 0;
};

static TimeControl timeControl;

static double loopTimeinmicros() {
    timeControl.now = micros();
    timeControl.dt = timeControl.now - timeControl.last;
    timeControl.last = timeControl.now;
    return timeControl.dt / 1000000;
};

static double loopTimeinMillis() {
    timeControl.now = millis();
    timeControl.dt = timeControl.now - timeControl.last;
    timeControl.last = timeControl.now;
    return timeControl.dt;
}

class movingAvg{
    private:
        double _scale = 1.0;
        std::deque<double> dq;
        double sum = 0;
        int _size = 100;
    
    public:
        //constructor
        movingAvg(int size = 100) : _size(size){}

        void setScale(double scale){
            _scale = scale;
        }
        
        //pop front
        void pop() {
            if (!dq.empty()) {
                sum -= dq.front();
                dq.pop_front();
            }
        }
        
        //push back
        void push(double x) {
            dq.push_back(x);
            sum += x;
            if (dq.size() > _size) {
                pop();
            }
        }
        
        
        void setSize(int size){
            _size = size;
            while (dq.size() > _size) {
                pop();
            }
        }
        
        double get_sum() {
            return sum;
        }
        
        double get_avg(){
            if (dq.empty()) return 0.0;
            return (sum / dq.size()) * _scale;
        }
        
};

#endif

