#ifndef VECTOR_H
#define VECTOR_H

class Vector {
public:
    double x, y;

    // Constructor to initialize the vector
    Vector() : x(0), y(0) {}
    Vector(double xVal, double yVal) : x(xVal), y(yVal) {}

    // Getter methods for x and y (you can also access directly)
    double getX() const { return x; }
    double getY() const { return y; }

    // Setter methods for x and y
    void setX(double xVal) { x = xVal; }
    void setY(double yVal) { y = yVal; }

    // Optional: Add other useful vector functions like magnitude, angle, etc.
    double magnitude() const {
        return sqrt(x * x + y * y);
    }

    double angle() const {
        return atan2(y, x);
    }
};

#endif  // VECTOR_H