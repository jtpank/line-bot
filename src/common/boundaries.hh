#pragma once

#include "common/messages.hh"

namespace sim {

bool boundary_is_keep_out(const Boundary &boundary);
bool point_in_boundary(const Boundary &boundary, Point point);
bool point_within_boundaries(const Mission &mission, Point point);
bool path_within_boundaries(const Mission &mission, Point start, Point end);
double distance_to_boundary_edges(const Mission &mission, Point point);
bool point_respects_boundary_buffer(const Mission &mission, Point point, double buffer_m);
Pose valid_start_pose(const Mission &mission);

}  // namespace sim
