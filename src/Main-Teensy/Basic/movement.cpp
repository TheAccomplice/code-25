#include "movement.h"
#include "config.h"
#include "vector.h"
#include "util.h"

#define FL_PWM_PIN 18
#define FL_INA_PIN 17
#define FL_INB_PIN 19
#define BL_PWM_PIN 14
#define BL_INA_PIN 15
#define BL_INB_PIN 13
#define FR_PWM_PIN 6
#define FR_INA_PIN 9
#define FR_INB_PIN 5
#define BR_PWM_PIN 11
#define BR_INA_PIN 10
#define BR_INB_PIN 12

//Motor multipliers to adjust speed/motor difference
#define FL_MULTIPLIER 1
#define FR_MULTIPLIER 1
#define BL_MULTIPLIER 1
#define BR_MULTIPLIER 1

#define SIN40 0.64278760F
#define SIN50 0.76604444F
#define COS40 0.76604444F
#define COS50 0.64278760F


Movement::Movement(){};

void Movement::initialize() {
    // initalize pins
    pinMode(FL_PWM_PIN, OUTPUT);
    pinMode(FL_INA_PIN, OUTPUT);
    pinMode(FL_INB_PIN, OUTPUT);
    pinMode(BL_PWM_PIN, OUTPUT);
    pinMode(BL_INA_PIN, OUTPUT);
    pinMode(BL_INB_PIN, OUTPUT);
    pinMode(FR_PWM_PIN, OUTPUT);
    pinMode(FR_INA_PIN, OUTPUT);
    pinMode(FR_INB_PIN, OUTPUT);
    pinMode(BR_PWM_PIN, OUTPUT);
    pinMode(BR_INA_PIN, OUTPUT);
    pinMode(BR_INB_PIN, OUTPUT);

    // Set PWM resolution to 10-bit (values range from 0 to 1023)
    analogWriteResolution(10);

    analogWriteFrequency(FL_PWM_PIN, 146484); 
    analogWriteFrequency(BL_PWM_PIN, 146484);
    analogWriteFrequency(FR_PWM_PIN, 146484);
    analogWriteFrequency(BR_PWM_PIN, 146484);
};

//Updates real time state
void Movement::updateParameters(double actualbearing, double actualdirection,
                                double actualvelocity) {
    _actualbearing = actualbearing;
    _actualdirection = actualdirection;
    _actualvelocity = actualvelocity;
};

//sets the robot's direction to a fixed (constant) value provided in argument
void Movement::setconstantDirection(Direction::constant params) {
    _targetdirection = params.value;
};

//calculates the direction the robot should move in to reach a specific point, corrects with PID
void Movement::setmovetoPointDirection(Direction::movetoPoint params) {
    _targetdirection =
        (Vector::fromPoint(params.destination) - params.robotCoordinate).angle;
};

//sets the direction the robot should move in to track a line (appropriate depth)
void Movement::setlinetrackDirection(Direction::linetrack params) {
    linetrackController.updateSetpoint(params.targetLineDepth);
    double correction = linetrackController.advance(params.lineDepth);
    // Serial.println(correction);

    //adjust direction robot should face relative to the line
    if (params.trackLeftwards == true) {
        _targetdirection = correction * 90 + params.angleBisector - 90;
    } else {
        _targetdirection = (correction * -90) + params.angleBisector + 90;
    }
}

void Movement::setconstantVelocity(Velocity::constant params) {
    _targetvelocity = params.value;
};

// Adjusts velocity dynamically to stop at a point
void Movement::setstopatPointVelocity(Velocity::stopatPoint params) {
    auto correction = stopatPointController.advance(params.errordistance);

    _targetvelocity = abs(correction);
    _targetvelocity =
        constrain(_targetvelocity, params.minSpeed, params.maxSpeed);
};

void Movement::setconstantBearing(Bearing::constant params) {
    _targetbearing = params.targetValue;
    _actualbearing = params.actualBearing;
};

//adjusts the robot's bearing as it moves toward a target destination smoothly
void Movement::setmoveBearingtoPoint(Bearing::moveBearingtoPoint params) {
    if (params.destination != _finalDestination) {
        _finalbearing = params.finalBearing;
        _initialbearing = _actualbearing;
        _initialrobotcoordinate = params.robotCoordinate;
    }
    _finalDestination = params.destination;
    double progress =
        (params.robotCoordinate - _initialrobotcoordinate).distance /      //distance covered from start
        (Vector::fromPoint(params.destination) - _initialrobotcoordinate)  //total distance to be covered from start
            .distance;

    _targetbearing =
        _initialbearing + progress * (_finalbearing - _initialbearing);    //gradual change of bearing according to progress
};

//Updates the PID for the bearing adjustment 
void Movement::setBearingSettings(double minV, double maxV, double KP,
                                  double KD, double KI) {
    bearingController.updateLimits(minV, maxV);
    bearingController.updateGains(KP, KD, KI);
};

//Calculates motor speed and send command to md
void Movement::drive(Point robotPosition){
    bearingController.updateSetpoint(_targetbearing);

    // Adjust movement based on current bearing angle
    if (abs(_targetbearing) <= 135){
        _movingbearing =
            bearingController.advance(clipAngleto180degrees(_actualbearing));
    } else {
        _movingbearing =
            bearingController.advance(clipAngleto360degrees(_actualbearing));
    }

    double x = sind(_targetdirection);
    double y = cosd(_targetdirection);

    //calculate how fast each motor should move
    const auto transformspeed = [this](double velocityDirection, //how much robot should move in certain position and how much it should rotate
                                       double angularComponent){
        //desired speed mutiply by the direction of the velocity (how speed should be adjusted on motor based on direction) + adjust for turning
        return (int)(_targetvelocity * velocityDirection + angularComponent);
    };

    double angularComponent = _movingbearing * ANGULAR_SCALE;

    double FLSpeed =
        transformspeed(x * SIN40 + y * COS50, angularComponent) * FL_MULTIPLIER;
    double FRSpeed = 
        transformspeed(x * -SIN40 + y * COS50, -angularComponent) * FR_MULTIPLIER;
    double BRSpeed = 
        transformspeed(x * SIN40 + y * COS50, -angularComponent) * BR_MULTIPLIER;
    double BLSpeed = 
        transformspeed(x * -SIN40 + y * COS50, angularComponent) * BL_MULTIPLIER;
    

    //If goes wrong way, reverse LOW/HIGH
    digitalWriteFast(FL_INA_PIN, FLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(FL_INB_PIN, FLSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(FR_INA_PIN, FRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(FR_INB_PIN, FRSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(BR_INA_PIN, BRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(BR_INB_PIN, BRSpeed > 0 ? HIGH : LOW);

    digitalWriteFast(BL_INA_PIN, BLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(BL_INB_PIN, BLSpeed > 0 ? HIGH : LOW);

    analogWrite(FL_PWM_PIN, constrain(FLSpeed, 0, 700));
    analogWrite(FR_PWM_PIN, constrain(FRSpeed, 0, 700));
    analogWrite(BL_PWM_PIN, constrain(BLSpeed, 0, 700));
    analogWrite(BR_PWM_PIN, constrain(BRSpeed, 0, 700));
}

//Sigmoid for smooth accel/deccel
double Movement::applySigmoid(double startSpeed, double endSpeed,
                              double progress, double constant) {
    const auto multiplier = 1 / (1 + powf(1200, constant * 2 * progress - 1));
    return startSpeed + (endSpeed - startSpeed) * multiplier;
};

std::vector<double> Movement::getmotorValues() {
    const auto x = sind(_targetdirection);
    const auto y = cosd(_targetdirection);

    const auto transformspeed = [this](double velocityDirection,
                                       double angularComponent) {
        return (int)(_targetvelocity * velocityDirection + angularComponent);
    };

    double angularComponent = _movingbearing * ANGULAR_SCALE;

    double FLSpeed = transformspeed(x * SIN40 + y * COS50, angularComponent);
    double FRSpeed = transformspeed(x * -SIN40 + y * COS50, -angularComponent);
    double BRSpeed = transformspeed(x * SIN40 + y * COS50, -angularComponent);
    double BLSpeed = transformspeed(x * -SIN40 + y * COS50, angularComponent);

    return {FLSpeed, FRSpeed, BLSpeed, BRSpeed};
};