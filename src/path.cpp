#include "path.h"

#include <stdio.h>

#include <optional>
#include <set>

#include "position.h"

static const int SPEED = 50;

void toAbsoluteCoordinates(PathVector &path) {
  for (int i = 0; i < path.size(); i++) {
    Position pos = path[i];

    pos.x = 50 * pos.x + 25;
    pos.y = 50 * pos.y + 25;

    path[i] = pos;
  }

  printf("size: %d\n", path.size());
}

void interpolatePath(PathVector &path, const Position &start,
                     const Position &end) {
  double d = start.distance(end);

  for (double n = 1; n < d; n++) {
    path.push_back(Position{/* .x = */ start.x + n / d * (end.x - start.x),
                            /* .y = */ start.y + n / d * (end.y - start.y),
                            /* .theta = */ SPEED});
  }
}

void generatePath(PathVector &path, std::vector<PathSegment> &result) {
  // Holds the last position we visited
  std::optional<Position> prev;

  // Holds all of the visited positions for the current path
  // pathVector includes interpolated points, and can have 700+ points
  // so we need separate storage for just the centerpoints of squares
  std::set<Position> visited{path.at(0)};

  // The current path we are building
  PathVector currentPath;

  for (int i = 0; i < path.size() - 1; i++) {
    Position start = path.at(i);
    Position end = path.at(i + 1);

    // check for 180deg turns
    // and check for already visited points, to prevent pure pursuit loops
    // we previously interpolated to this point (prev -> start)
    if ((prev.has_value() && prev.value().equals(end)) ||
        visited.find(end) != visited.end()) {
      // set the last segment's speed to 0 to indicate end of path
      if (!currentPath.empty()) {
        Position &last = currentPath.back();
        last.theta = 0;
      }

      // add the current path and the angle turn command to the result
      result.push_back({0, start.angle(end)});

      // reset current path
      currentPath = std::vector<Position>();
      visited.clear();
      visited.insert(end);
    }

    // interpolate between current and next
    // if we did 180deg, we are now new path (oldpath, turn, start -> end)
    interpolatePath(currentPath, start, end);

    // if last part, set speed 0
    currentPath.push_back({end.x, end.y, SPEED});

    visited.insert(end);
    prev = start;
  }

  // push in last segment
  // and set speed to 0 to indicate end of path
  if (!currentPath.empty()) {
    Position &last = currentPath.back();
    last.theta = 0;
  }

  result.push_back({0, currentPath});
}
