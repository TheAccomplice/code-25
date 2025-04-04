#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <vector>

#include "pid.h"
#include "vector.h"  // Assuming this includes the Vector class we created earlier

#define ANGULAR_SCALE 1.3


namespace Direction {

// Move in a specific direction
struct Constant {
  double value;
};
struct MoveToPoint {
  Vector robotCoordinate;
  Point destination;
};
struct LineTrack {
  // Depth at which robot is tracking a line, adjusted to stay on the line
  double lineDepth;
  double angleBisector;
  double targetLineDepth = 0.5;
  bool trackLeftwards = true;  // Track left of an angle bisector from the line
};
}  // namespace Direction

namespace Velocity {
struct Constant {
  double value;
};
struct StopAtPoint {
  double errorDistance;
  double minSpeed;
  double maxSpeed;
};
}  // namespace Velocity

namespace Bearing {
struct Constant {
  double targetValue;
  double actualBearing;
};
struct MoveBearingToPoint {
  Vector robotCoordinate;
  Point destination;
  double finalBearing;
};
struct Absolute {
  double angle;
};
}  // namespace Bearing

class Movement {
 public:
  Movement();

  void updateParameters(double actualbearing, double actualdirection,
                        double actualvelocity);

  void initialize();
  // Set relevant parameters
  void setConstantDirection(Direction::Constant params);
  void setMoveToPointDirection(Direction::MoveToPoint params);
  void setLineTrackDirection(Direction::LineTrack params);

  void setConstantVelocity(Velocity::Constant params);
  void setStopAtPointVelocity(Velocity::StopAtPoint params);

  void setConstantBearing(Bearing::Constant params);
  void setMoveBearingToPoint(Bearing::MoveBearingToPoint params);
  void setBearingSettings(double min, double max, double KP, double KD,
                          double KI);

  // PID Controllers
  PID bearingController = PID(0.0, -600, 600, 4, 40, 0.0, 1);
  PID directionController = PID(0.0, -90, 90, 1.5, 0, 0, 1);
  PID stopAtPointController = PID(0.0, -360, 360, 1.5, 0.01, 0.1, 1);
  PID lineTrackController = PID(0.0, -0.3, 0.3, 0.5, 0, 0, 1);

  void drive(Point robotPosition);
  double applySigmoid(double startSpeed, double endSpeed, double progress,
                      double constant);

  std::vector<double> getMotorValues();

  // Add the missing setBearing method to set _lastBearing
  void setBearing(double bearing) { _lastbearing = bearing; }

 private:
  // Parameters
  double _targetdirection;
  double _targetbearing;
  double _targetvelocity;

  // Actual moving parameters
  double _movingdirection;
  double _movingbearing;
  double _movingvelocity;

  // Past values
  double _lastdirection;
  double _lastbearing;
  double _lastvelocity;

  // For Bearing::MoveBearingToPoint and MoveToPoint
  Point _finalDestination;
  double _initialbearing;
  double _finalbearing;
  Vector _initialrobotcoordinate;

  // Actual parameters
  double _actualVelocity;
  double _actualbearing;
  double _actualdirection;
};

#endif