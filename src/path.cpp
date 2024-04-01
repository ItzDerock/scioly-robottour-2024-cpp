#include "path.h"
#include "position.h"

#include <stdio.h>

static const int SPEED = 50;

void toAbsoluteCoordinates(PathVector &path) {
  // for (Position pos : path) {
  for (int i = 0; i < path.size(); i++) {
    Position pos = path[i];

    pos.x = 50 * pos.x + 25;
    pos.y = 50 * pos.y + 25;

    path[i] = pos;
  }

  printf("size: %d\n", path.size());
}

void interpolateAbsolutePath(PathVector &path, PathVector &result) {
  for (int i = 0; i < path.size() - 1; i++) {
    Position start = path.at(i);
    Position end = path.at(i + 1);

    double d = start.distance(end);

    // result.push_back(Position{start.x, start.y, SPEED});

    for (double n = 1; n < d; n++) {
      result.push_back(
          Position{/* .x = */ start.x + n / d * (end.x - start.x),
                   /* .y = */ start.y + n / d * (end.y - start.y),
                   /* .theta = */ SPEED});
    }

    if (i + 1 == path.size() - 1) {
      result.push_back(Position{end.x, end.y, 0});
    } else {
      result.push_back(Position{end.x, end.y, SPEED});
    }
  }
}
