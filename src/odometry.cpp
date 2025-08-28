#include "odometry.h"
#include <cmath>
#include <iterator>
#include <numeric>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) = (wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res = {0.0, 0.0}; // angle_deg, time_sec

  if (path.size() < 2)
    return res; // no movement if only one point

  double total_dist = 0.0;
  double total_angle = 0.0;

  // initial heading
  double prev_angle = angle(path[0].first, path[0].second,
                            path[1].first, path[1].second);

  for (size_t i = 1; i < path.size(); i++) {
    // distance between consecutive points
    total_dist += distance(path[i - 1].first, path[i - 1].second,
                           path[i].first, path[i].second);

    // check heading change (only if not at last point)
    if (i + 1 < path.size()) {
      double curr_angle =
          angle(path[i].first, path[i].second, path[i + 1].first, path[i + 1].second);

      double delta = curr_angle - prev_angle;

      // normalize delta to [-180, 180]
      while (delta > 180) delta -= 360;
      while (delta < -180) delta += 360;

      total_angle += fabs(delta);
      prev_angle = curr_angle;
    }
  }

  // compute time
  double total_time = total_dist / linear_vel;

  res.angle_deg = total_angle;
  res.time_sec = total_time;
  return res;
}

