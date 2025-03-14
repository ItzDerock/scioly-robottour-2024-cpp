#pragma once

#include <math.h>

struct Position {
  double x;
  double y;
  double theta;

  int getDegrees() const { return (int)(theta * 180 / M_PI); }
  Position(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  // equal
  bool equals(const Position &other, bool checkTheta = true) const {
    return this->x == other.x && this->y == other.y &&
           (checkTheta ? this->theta == other.theta : true);
  }

  bool operator==(const Position &other) const { return equals(other); }

  // subtract operator
  Position operator-(const Position &other) const {
    return Position(x - other.x, y - other.y, theta - other.theta);
  }

  // add operator
  Position operator+(const Position &other) const {
    return Position(x + other.x, y + other.y, theta + other.theta);
  }

  // multiply operator
  double operator*(const Position &other) const {
    return this->x * other.x + this->y * other.y;
  }

  Position operator*(const double &other) const {
    return Position(x * other, y * other, theta);
  }

  double distance(const Position &other) const {
    return std::hypot(this->x - other.x, this->y - other.y);
  }

  Position lerp(Position other, double t) const {
    return Position(this->x + (other.x - this->x) * t,
                    this->y + (other.y - this->y) * t, this->theta);
  }

  float angle(Position other) const {
    return std::atan2(other.y - this->y, other.x - this->x);
  }
};
