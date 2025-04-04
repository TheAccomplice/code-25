#include "movement.h"
#include "vector.h"
#include "util.h"

// Motor pin definitions
#define FL_PWM_PIN 18
#define FL_INA_PIN 17
#define FL_INB_PIN 19
#define BL_PWM_PIN 14
#define BL_INA_PIN 13
#define BL_INB_PIN 15
#define FR_PWM_PIN 6 
#define FR_INA_PIN 9 //fr check direction
#define FR_INB_PIN 5
#define BR_PWM_PIN 11
#define BR_INA_PIN 10
#define BR_INB_PIN 12

// Motor multipliers
#define FL_MULTIPLIER 1.0
#define FR_MULTIPLIER 1.0
#define BL_MULTIPLIER 1.0
#define BR_MULTIPLIER 1.0

#define SIN40 0.64278760F
#define COS40 0.76604444F
#define SIN50 0.76604444F
#define COS50 0.64278760F

// Constructor
Movement::Movement() {}

// Initialize pins and PWM
void Movement::initialize() {
    int motorPins[] = {FL_PWM_PIN, FL_INA_PIN, FL_INB_PIN, BL_PWM_PIN, BL_INA_PIN, BL_INB_PIN,
                       FR_PWM_PIN, FR_INA_PIN, FR_INB_PIN, BR_PWM_PIN, BR_INA_PIN, BR_INB_PIN};
    for (int pin : motorPins) pinMode(pin, OUTPUT);

    analogWriteResolution(10);
    int pwmFrequency = 146484;
    analogWriteFrequency(FL_PWM_PIN, pwmFrequency);
    analogWriteFrequency(BL_PWM_PIN, pwmFrequency);
    analogWriteFrequency(FR_PWM_PIN, pwmFrequency);
    analogWriteFrequency(BR_PWM_PIN, pwmFrequency);
}

// Update state
void Movement::updateParameters(double bearing, double direction, double velocity) {
    _actualbearing = bearing;
    _actualdirection = direction;
    _actualVelocity = velocity;
}

void Movement::setConstantDirection(Direction::Constant params) {
    _targetdirection = params.value;
}

void Movement::setMoveToPointDirection(Direction::MoveToPoint params) {
    _targetdirection =
        (Vector::fromPoint(params.destination) - params.robotCoordinate).angle;
}

void Movement::setLineTrackDirection(Direction::LineTrack params) {
    lineTrackController.updateSetpoint(params.targetLineDepth);
    double correction = lineTrackController.advance(params.lineDepth);
    _targetdirection = params.trackLeftwards ? (correction * 90 + params.angleBisector - 90)
                                             : (-correction * 90 + params.angleBisector + 90);
}

void Movement::setConstantVelocity(Velocity::Constant params) {
    _targetvelocity = params.value;
}

void Movement::setStopAtPointVelocity(Velocity::StopAtPoint params) {
    double correction = stopAtPointController.advance(params.errorDistance);
    _targetvelocity = constrain(abs(correction), params.minSpeed, params.maxSpeed);
}

void Movement::setConstantBearing(Bearing::Constant params) {
    _targetbearing = params.targetValue;
    _actualbearing = params.actualBearing;
}

void Movement::setMoveBearingToPoint(Bearing::MoveBearingToPoint params) {
    if (!(params.destination == _finalDestination)) {
        _finalbearing = params.finalBearing;
        _initialbearing = _actualbearing;
        _initialrobotcoordinate = params.robotCoordinate;
    }
    _finalDestination = params.destination;

    Vector current(params.robotCoordinate);
    Vector initial(_initialrobotcoordinate);
    Vector final(Vector::fromPoint(params.destination));

    double totalDistance = (final - initial).distance;
    double progress = totalDistance != 0 ? (current - initial).distance / totalDistance : 1.0;
    progress = constrain(progress, 0.0, 1.0);
    _targetbearing = _initialbearing + progress * (_finalbearing - _initialbearing);
}

void Movement::setBearingSettings(double minV, double maxV, double KP, double KD, double KI) {
    bearingController.updateLimits(minV, maxV);
    bearingController.updateGains(KP, KD, KI);
}

void Movement::drive(Point robotPosition) {
    bearingController.updateSetpoint(_targetbearing);
    _movingbearing = bearingController.advance(abs(_targetbearing) <= 135
                          ? clipAngleto180degrees(_actualbearing)
                          : clipAngleto360degrees(_actualbearing));

    double x = sind(_targetdirection);
    double y = cosd(_targetdirection);
    double angularComponent = _movingbearing * ANGULAR_SCALE;

    auto transformSpeed = [&](double velocityComponent, double angular) {
        return _targetvelocity * velocityComponent + angular;
    };

    double flSpeed = transformSpeed(x * SIN40 + y * COS50, angularComponent) * FL_MULTIPLIER;
    double frSpeed = transformSpeed(-x * SIN40 + y * COS50, -angularComponent) * FR_MULTIPLIER;
    double brSpeed = transformSpeed(x * SIN40 + y * COS50, -angularComponent) * BR_MULTIPLIER;
    double blSpeed = transformSpeed(-x * SIN40 + y * COS50, angularComponent) * BL_MULTIPLIER;

    double speeds[] = {flSpeed, frSpeed, brSpeed, blSpeed};
    int motorINAPins[] = {FL_INA_PIN, FR_INA_PIN, BR_INA_PIN, BL_INA_PIN};
    int motorINBPins[] = {FL_INB_PIN, FR_INB_PIN, BR_INB_PIN, BL_INB_PIN};
    int motorPWMPins[] = {FL_PWM_PIN, FR_PWM_PIN, BR_PWM_PIN, BL_PWM_PIN};

    for (int i = 0; i < 4; i++) {
        digitalWriteFast(motorINAPins[i], speeds[i] > 0 ? LOW : HIGH);
        digitalWriteFast(motorINBPins[i], speeds[i] > 0 ? HIGH : LOW);
        analogWrite(motorPWMPins[i], constrain(abs((int)speeds[i]), 0, 700));
    }
}

double Movement::applySigmoid(double startSpeed, double endSpeed, double progress, double constant) {
    return startSpeed + (endSpeed - startSpeed) * (1.0 / (1.0 + powf(1200, constant * 2 * progress - 1)));
}

std::vector<double> Movement::getMotorValues() {
    double x = sind(_targetdirection);
    double y = cosd(_targetdirection);
    double angularComponent = _movingbearing * ANGULAR_SCALE;

    auto transformSpeed = [&](double velocityComponent, double angular) {
        return _targetvelocity * velocityComponent + angular;
    };

    return {
        transformSpeed(x * SIN40 + y * COS50, angularComponent),
        transformSpeed(-x * SIN40 + y * COS50, -angularComponent),
        transformSpeed(x * SIN40 + y * COS50, -angularComponent),
        transformSpeed(-x * SIN40 + y * COS50, angularComponent)
    };
}