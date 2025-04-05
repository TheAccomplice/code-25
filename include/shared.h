#ifndef SHARED_H
#define SHARED_H

#include "vector.h"
#include <deque>

struct ProcessedValues {
    //relative to robot
    Vector ball_relativeposition;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    //relative to field, not used?
    Vector ball_actualposition;
    Vector yellowgoal_actualposition;
    Vector bluegoal_actualposition;
    Vector robot_position;

    int ballExists = 0;
    int ball_in_catchment = 0;
    int yellowgoal_exists = 0;
    int bluegoal_exists = 0;
    int lidarDistance[4];
    double lidarConfidence[4];
    double bearing_relative_to_field;
    int is_ball_in_catchment = 0;
    int relativeBearing;
    int onLine = 0;
    float angleBisector;
    float depthinLine;
    //double depth_in_line;
    // double past_true_x_left = 0;
    // double past_true_x_right = 0;
    // double past_true_y_front = 0;
    // double past_true_y_back = 0;
};

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
        #include <deque>

        #include <deque>

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

extern movingAvg whiteAvg[15];
extern movingAvg greenAvg[15];

#endif

