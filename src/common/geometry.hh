#pragma once

#include <algorithm>
#include <cmath>

namespace sim {

constexpr double kPi = 3.14159265358979323846;

struct Point {
    double x{0.0};
    double y{0.0};
};

struct Pose {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

double clamp(double value, double low, double high);
double wrap_angle(double angle);
double distance(Point a, Point b);
double segment_length(Point a, Point b);
double heading_to(Point from, Point to);
double signed_lateral_error(Point start, Point end, Point point);
double projection_ratio(Point start, Point end, Point point);
double distance_to_segment(Point start, Point end, Point point);
bool segments_intersect(Point a, Point b, Point c, Point d);
Point offset_from_segment(Point start, Point end, double ratio, double offset);

}  // namespace sim
