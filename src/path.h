#pragma once

#include <variant>
#include <vector>

#include "position.h"

typedef std::vector<Position> PathVector;

struct PathSegment {
  float shouldFinishAt;
  std::variant<PathVector, float> data;
};

void toAbsoluteCoordinates(PathVector &path);
void generatePath(PathVector &path, std::vector<PathSegment> &result);
