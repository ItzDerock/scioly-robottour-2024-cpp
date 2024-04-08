#pragma once

#include "position.h"
#include <vector>
#include <variant>

typedef std::vector<Position> PathVector;

struct PathSegment {
  float shouldFinishAt;
  std::variant<PathVector, float> data;
};

void toAbsoluteCoordinates(PathVector &path);
void interpolateAbsolutePath(PathVector &path, std::vector<PathSegment> &result);
