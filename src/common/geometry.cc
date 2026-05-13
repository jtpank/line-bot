#include "common/geometry.hh"

namespace sim {

double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double distance(Point a, Point b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

double segment_length(Point a, Point b) {
    return distance(a, b);
}

double heading_to(Point from, Point to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

double signed_lateral_error(Point start, Point end, Point point) {
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double len = std::hypot(vx, vy);
    if (len <= 1e-9) {
        return distance(start, point);
    }
    const double px = point.x - start.x;
    const double py = point.y - start.y;
    return (vx * py - vy * px) / len;
}

double projection_ratio(Point start, Point end, Point point) {
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double len_sq = vx * vx + vy * vy;
    if (len_sq <= 1e-9) {
        return 0.0;
    }
    const double px = point.x - start.x;
    const double py = point.y - start.y;
    return (px * vx + py * vy) / len_sq;
}

double distance_to_segment(Point start, Point end, Point point) {
    const double ratio = clamp(projection_ratio(start, end, point), 0.0, 1.0);
    Point closest{
        .x = start.x + (end.x - start.x) * ratio,
        .y = start.y + (end.y - start.y) * ratio,
    };
    return distance(closest, point);
}

namespace {

double orientation(Point a, Point b, Point c) {
    const double value = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
    if (std::abs(value) < 1e-9) {
        return 0.0;
    }
    return value > 0.0 ? 1.0 : 2.0;
}

bool on_segment(Point a, Point b, Point c) {
    return b.x <= std::max(a.x, c.x) + 1e-9 &&
           b.x + 1e-9 >= std::min(a.x, c.x) &&
           b.y <= std::max(a.y, c.y) + 1e-9 &&
           b.y + 1e-9 >= std::min(a.y, c.y);
}

}  // namespace

bool segments_intersect(Point a, Point b, Point c, Point d) {
    const double o1 = orientation(a, b, c);
    const double o2 = orientation(a, b, d);
    const double o3 = orientation(c, d, a);
    const double o4 = orientation(c, d, b);

    if (o1 != o2 && o3 != o4) {
        return true;
    }
    if (o1 == 0.0 && on_segment(a, c, b)) {
        return true;
    }
    if (o2 == 0.0 && on_segment(a, d, b)) {
        return true;
    }
    if (o3 == 0.0 && on_segment(c, a, d)) {
        return true;
    }
    if (o4 == 0.0 && on_segment(c, b, d)) {
        return true;
    }
    return false;
}

Point offset_from_segment(Point start, Point end, double ratio, double offset) {
    ratio = clamp(ratio, 0.0, 1.0);
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double len = std::hypot(vx, vy);
    if (len <= 1e-9) {
        return start;
    }
    const double nx = -vy / len;
    const double ny = vx / len;
    return Point{
        .x = start.x + vx * ratio + nx * offset,
        .y = start.y + vy * ratio + ny * offset,
    };
}

}  // namespace sim
