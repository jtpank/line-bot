#include "common/paint_arm.hh"

#include <cmath>

namespace sim {

Point paint_arm_world(Pose robot_pose, const Mission &mission) {
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    return Point{
        .x = robot_pose.x + mission.paint_arm_forward_m * cos_yaw - mission.paint_arm_left_m * sin_yaw,
        .y = robot_pose.y + mission.paint_arm_forward_m * sin_yaw + mission.paint_arm_left_m * cos_yaw,
    };
}

Point robot_center_for_paint_point(Point paint_point, double line_heading_rad, const Mission &mission) {
    const double cos_heading = std::cos(line_heading_rad);
    const double sin_heading = std::sin(line_heading_rad);
    return Point{
        .x = paint_point.x - mission.paint_arm_forward_m * cos_heading + mission.paint_arm_left_m * sin_heading,
        .y = paint_point.y - mission.paint_arm_forward_m * sin_heading - mission.paint_arm_left_m * cos_heading,
    };
}

Segment robot_center_segment_for_paint_segment(const Segment &segment, const Mission &mission) {
    const double heading = heading_to(segment.start, segment.end);
    return Segment{
        .id = segment.id,
        .start = robot_center_for_paint_point(segment.start, heading, mission),
        .end = robot_center_for_paint_point(segment.end, heading, mission),
    };
}

}  // namespace sim
