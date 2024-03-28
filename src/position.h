#pragma once

#include <cmath>

struct Position {
  double x;
  double y;
  double theta;

  int getDegrees() const { return (int)(theta * 180 / M_PI); }
  Position(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  // subtract operator
  Position operator-(const Position& other) const {
    return Position(x - other.x, y - other.y, theta - other.theta);
  }

  // add operator
  Position operator+(const Position& other) const {
    return Position(x + other.x, y + other.y, theta + other.theta);
  }

  // multiply operator
  double operator*(const Position& other) const {
    return this->x * other.x + this->y * other.y;
  }

  Position operator*(const double& other) const {
    return Position(x * other, y * other, theta);
  }

  double distance(const Position& other) const {
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
