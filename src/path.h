#pragma once

#include "position.h"
#include <vector>

typedef std::vector<Position> PathVector;

void toAbsoluteCoordinates(PathVector &path);
void interpolateAbsolutePath(PathVector &path, PathVector &result);
