#include "path.h"

#include <set>
#include <stdio.h>

#include <optional>

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
  std::optional<Position> prev;
  PathVector currentPath;

  for (int i = 0; i < path.size() - 1; i++) {
    Position start = path.at(i);
    Position end = path.at(i + 1);

    // interpolate between current and next
    interpolatePath(currentPath, start, end);

    // if last part, set speed 0
    currentPath.push_back(
        Position{end.x, end.y, (i + 1 == path.size() - 1) ? 0 : SPEED});

    // check if 180deg turn
    if (i < path.size() - 2 && path.at(i + 2).equals(start)) {
      currentPath.pop_back();
      currentPath.push_back({end.x, end.y, 0});

      result.push_back({
          0, // field not used atm
          currentPath,
      });
      result.push_back({0, start.angle(end)});

      currentPath = std::vector<Position>();
    }

    prev = start;
  }

  // push in last segment
  result.push_back({0, currentPath});
}
