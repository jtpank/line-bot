#pragma once

#include "common/messages.hh"

namespace sim {

Point paint_arm_world(Pose robot_pose, const Mission &mission);
Point robot_center_for_paint_point(Point paint_point, double line_heading_rad, const Mission &mission);
Segment robot_center_segment_for_paint_segment(const Segment &segment, const Mission &mission);

}  // namespace sim
