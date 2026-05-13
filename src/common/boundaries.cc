#include "common/boundaries.hh"

#include <cmath>
#include <limits>

namespace sim {
namespace {

constexpr double kBoundaryToleranceM = 0.04;

bool point_on_segment(Point start, Point end, Point point) {
    const double ratio = projection_ratio(start, end, point);
    return ratio >= -1e-6 && ratio <= 1.0 + 1e-6 &&
           distance_to_segment(start, end, point) <= kBoundaryToleranceM;
}

bool has_keep_in_boundary(const Mission &mission) {
    for (const auto &boundary : mission.boundaries) {
        if (!boundary_is_keep_out(boundary)) {
            return true;
        }
    }
    return false;
}

Point polygon_centroid(const Boundary &boundary) {
    Point centroid;
    if (boundary.vertices.empty()) {
        return centroid;
    }
    for (const auto &vertex : boundary.vertices) {
        centroid.x += vertex.x;
        centroid.y += vertex.y;
    }
    centroid.x /= static_cast<double>(boundary.vertices.size());
    centroid.y /= static_cast<double>(boundary.vertices.size());
    return centroid;
}

}  // namespace

bool boundary_is_keep_out(const Boundary &boundary) {
    return boundary.kind == "keep_out";
}

bool point_in_boundary(const Boundary &boundary, Point point) {
    if (boundary.vertices.size() < 3) {
        return false;
    }

    bool inside = false;
    for (size_t i = 0, j = boundary.vertices.size() - 1; i < boundary.vertices.size(); j = i++) {
        const Point a = boundary.vertices[j];
        const Point b = boundary.vertices[i];
        if (point_on_segment(a, b, point)) {
            return true;
        }
        const bool crosses_y = (a.y > point.y) != (b.y > point.y);
        if (crosses_y) {
            const double x_at_y = (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
            if (point.x < x_at_y) {
                inside = !inside;
            }
        }
    }
    return inside;
}

bool point_within_boundaries(const Mission &mission, Point point) {
    bool inside_keep_in = !has_keep_in_boundary(mission);
    for (const auto &boundary : mission.boundaries) {
        const bool inside = point_in_boundary(boundary, point);
        if (boundary_is_keep_out(boundary) && inside) {
            return false;
        }
        if (!boundary_is_keep_out(boundary) && inside) {
            inside_keep_in = true;
        }
    }
    return inside_keep_in;
}

bool path_within_boundaries(const Mission &mission, Point start, Point end) {
    const int samples = std::max(2, static_cast<int>(std::ceil(distance(start, end) / 0.2)));
    for (int index = 0; index <= samples; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(samples);
        const Point sample{
            .x = start.x + (end.x - start.x) * ratio,
            .y = start.y + (end.y - start.y) * ratio,
        };
        if (!point_within_boundaries(mission, sample)) {
            return false;
        }
    }
    return true;
}

double distance_to_boundary_edges(const Mission &mission, Point point) {
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto &boundary : mission.boundaries) {
        for (size_t index = 0; index < boundary.vertices.size(); ++index) {
            const Point start = boundary.vertices[index];
            const Point end = boundary.vertices[(index + 1) % boundary.vertices.size()];
            minimum = std::min(minimum, distance_to_segment(start, end, point));
        }
    }
    return minimum;
}

bool point_respects_boundary_buffer(const Mission &mission, Point point, double buffer_m) {
    if (!point_within_boundaries(mission, point)) {
        return false;
    }
    if (mission.boundaries.empty() || buffer_m <= 0.0) {
        return true;
    }
    return distance_to_boundary_edges(mission, point) >= buffer_m;
}

Pose valid_start_pose(const Mission &mission) {
    const Point home{.x = mission.home.x, .y = mission.home.y};
    if (point_within_boundaries(mission, home)) {
        return mission.home;
    }

    for (const auto &segment : mission.segments) {
        if (point_within_boundaries(mission, segment.start)) {
            return Pose{.x = segment.start.x, .y = segment.start.y, .yaw = heading_to(segment.start, segment.end)};
        }
    }

    for (const auto &boundary : mission.boundaries) {
        if (!boundary_is_keep_out(boundary)) {
            const Point centroid = polygon_centroid(boundary);
            if (point_within_boundaries(mission, centroid)) {
                return Pose{.x = centroid.x, .y = centroid.y, .yaw = mission.home.yaw};
            }
        }
    }

    return mission.home;
}

}  // namespace sim
