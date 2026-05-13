#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <vector>

#include "common/boundaries.hh"
#include "common/dora_node.hh"
#include "common/geometry.hh"
#include "common/messages.hh"
#include "common/paint_arm.hh"

namespace {

struct ActiveTarget {
    std::string line_id;
    bool entry_is_start{true};
    bool staged{false};
    bool entered{false};
};

struct SelectedTarget {
    sim::Segment segment;
    sim::Segment center_segment;
    bool entry_is_start{true};
};

struct LineFeasibility {
    bool feasible{true};
    std::string reason;
    sim::Segment center_segment;
};

sim::Point endpoint(const sim::Segment &segment, bool start) {
    return start ? segment.start : segment.end;
}

sim::Point staging_point(const sim::Segment &segment, bool entry_is_start, const sim::Mission &mission) {
    const sim::Point entry = endpoint(segment, entry_is_start);
    const sim::Point exit = endpoint(segment, !entry_is_start);
    const double length = sim::segment_length(entry, exit);
    if (length <= 1e-9) {
        return entry;
    }

    const double approach_m = std::max(3.0, mission.avoid_clearance_m * 4.0);
    const double ux = (exit.x - entry.x) / length;
    const double uy = (exit.y - entry.y) / length;
    return sim::Point{
        .x = entry.x - ux * approach_m,
        .y = entry.y - uy * approach_m,
    };
}

sim::Point paint_point_from_state(const sim::RobotState &state, const sim::Mission &mission) {
    return sim::paint_arm_world(sim::Pose{.x = state.x, .y = state.y, .yaw = state.yaw}, mission);
}

sim::Point boundary_centroid(const sim::Boundary &boundary) {
    sim::Point center;
    if (boundary.vertices.empty()) {
        return center;
    }
    for (const auto &vertex : boundary.vertices) {
        center.x += vertex.x;
        center.y += vertex.y;
    }
    center.x /= static_cast<double>(boundary.vertices.size());
    center.y /= static_cast<double>(boundary.vertices.size());
    return center;
}

std::optional<sim::Segment> find_segment(const sim::Mission &mission, const std::string &line_id) {
    for (const auto &segment : mission.segments) {
        if (segment.id == line_id) {
            return segment;
        }
    }
    return std::nullopt;
}

bool near_segment_endpoint(const sim::Segment &segment, sim::Point point, double clearance) {
    return sim::distance(point, segment.start) < clearance || sim::distance(point, segment.end) < clearance;
}

bool path_drives_over_segment(const sim::Mission &mission,
                              const sim::Segment &segment,
                              sim::Point current,
                              sim::Point target) {
    const int samples = std::max(2, static_cast<int>(std::ceil(sim::distance(current, target) / 0.1)));
    for (int index = 0; index <= samples; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(samples);
        const sim::Point sample{
            .x = current.x + (target.x - current.x) * ratio,
            .y = current.y + (target.y - current.y) * ratio,
        };
        const double along = sim::projection_ratio(segment.start, segment.end, sample);
        if (along <= 0.05 || along >= 0.95) {
            continue;
        }
        if (std::abs(sim::signed_lateral_error(segment.start, segment.end, sample)) <=
            mission.completion_tolerance_m) {
            return true;
        }
    }
    return false;
}

bool path_drives_over_completed(const sim::Mission &mission,
                                const sim::PaintReport &report,
                                sim::Point current,
                                sim::Point target,
                                const std::string &active_line_id) {
    for (const auto &segment : mission.segments) {
        if (segment.id == active_line_id || !report.completed.contains(segment.id)) {
            continue;
        }
        if (path_drives_over_segment(mission, segment, current, target)) {
            return true;
        }
    }
    return false;
}

std::optional<sim::Segment> blocking_completed_segment(const sim::Mission &mission,
                                                       const sim::PaintReport &report,
                                                       sim::Point current,
                                                       sim::Point target,
                                                       const std::string &active_line_id) {
    for (const auto &segment : mission.segments) {
        if (segment.id == active_line_id || !report.completed.contains(segment.id)) {
            continue;
        }
        const double clearance = mission.avoid_clearance_m;
        if (path_drives_over_segment(mission, segment, current, target)) {
            return segment;
        }

        const bool endpoint_handoff = near_segment_endpoint(segment, current, clearance) ||
                                      near_segment_endpoint(segment, target, clearance);
        if (endpoint_handoff) {
            continue;
        }

        if (sim::segments_intersect(current, target, segment.start, segment.end) ||
            sim::distance_to_segment(segment.start, segment.end, target) < clearance) {
            return segment;
        }
    }
    return std::nullopt;
}

std::optional<sim::Point> avoidance_waypoint(const sim::Mission &mission,
                                             const sim::PaintReport &report,
                                             sim::Point current,
                                             sim::Point target,
                                             const std::string &active_line_id) {
    const auto blocking = blocking_completed_segment(mission, report, current, target, active_line_id);
    if (!blocking) {
        return std::nullopt;
    }

    const double length = sim::segment_length(blocking->start, blocking->end);
    if (length <= 1e-9) {
        return blocking->start;
    }

    const double ux = (blocking->end.x - blocking->start.x) / length;
    const double uy = (blocking->end.y - blocking->start.y) / length;
    const double nx = -uy;
    const double ny = ux;
    const double minimum_escape_m = std::max(0.15, mission.completion_tolerance_m * 1.3);
    const std::vector<double> clearances{
        mission.avoid_clearance_m * 1.7,
        mission.avoid_clearance_m,
        mission.avoid_clearance_m * 0.7,
        minimum_escape_m,
    };

    std::vector<sim::Point> candidates;
    for (const double clearance : clearances) {
        if (clearance <= 1e-6) {
            continue;
        }
        candidates.push_back(sim::Point{
            .x = blocking->start.x - ux * clearance,
            .y = blocking->start.y - uy * clearance,
        });
        candidates.push_back(sim::Point{
            .x = blocking->end.x + ux * clearance,
            .y = blocking->end.y + uy * clearance,
        });
    }

    const double side_clearance = std::max(mission.avoid_clearance_m * 1.7, mission.completion_tolerance_m * 2.0);
    for (const double ratio : {0.2, 0.5, 0.8}) {
        const sim::Point center{
            .x = blocking->start.x + (blocking->end.x - blocking->start.x) * ratio,
            .y = blocking->start.y + (blocking->end.y - blocking->start.y) * ratio,
        };
        candidates.push_back(sim::Point{.x = center.x + nx * side_clearance, .y = center.y + ny * side_clearance});
        candidates.push_back(sim::Point{.x = center.x - nx * side_clearance, .y = center.y - ny * side_clearance});
    }

    double best_score = std::numeric_limits<double>::infinity();
    std::optional<sim::Point> best;
    for (const auto &candidate : candidates) {
        if (!sim::path_within_boundaries(mission, current, candidate) ||
            path_drives_over_completed(mission, report, current, candidate, active_line_id)) {
            continue;
        }

        double score = sim::distance(current, candidate) + sim::distance(candidate, target);
        if (!sim::path_within_boundaries(mission, candidate, target)) {
            score += 100.0;
        }
        if (path_drives_over_completed(mission, report, candidate, target, active_line_id)) {
            score += 100.0;
        }
        if (score < best_score) {
            best_score = score;
            best = candidate;
        }
    }
    return best;
}

std::optional<sim::Point> boundary_avoidance_waypoint(const sim::Mission &mission,
                                                      const sim::PaintReport &report,
                                                      sim::Point current,
                                                      sim::Point target,
                                                      const std::string &active_line_id) {
    const double clearance = std::max(mission.avoid_clearance_m, mission.track_width_m * 0.75);
    std::vector<sim::Point> candidates;

    for (const auto &boundary : mission.boundaries) {
        if (!sim::boundary_is_keep_out(boundary) || boundary.vertices.size() < 3) {
            continue;
        }
        const sim::Point center = boundary_centroid(boundary);
        for (const auto &vertex : boundary.vertices) {
            const double dx = vertex.x - center.x;
            const double dy = vertex.y - center.y;
            const double length = std::hypot(dx, dy);
            if (length <= 1e-9) {
                continue;
            }
            candidates.push_back(sim::Point{
                .x = vertex.x + dx / length * clearance,
                .y = vertex.y + dy / length * clearance,
            });
        }
    }

    double best_score = std::numeric_limits<double>::infinity();
    std::optional<sim::Point> best;
    for (const auto &candidate : candidates) {
        if (!sim::point_within_boundaries(mission, candidate) ||
            !sim::path_within_boundaries(mission, current, candidate) ||
            path_drives_over_completed(mission, report, current, candidate, active_line_id)) {
            continue;
        }

        double score = sim::distance(current, candidate) + sim::distance(candidate, target);
        if (!sim::path_within_boundaries(mission, candidate, target)) {
            score += 100.0;
        }
        if (path_drives_over_completed(mission, report, candidate, target, active_line_id)) {
            score += 100.0;
        }
        if (score < best_score) {
            best_score = score;
            best = candidate;
        }
    }
    return best;
}

std::optional<sim::Point> navigation_waypoint(const sim::Mission &mission,
                                              const sim::PaintReport &report,
                                              sim::Point current,
                                              sim::Point target,
                                              const std::string &active_line_id) {
    if (sim::path_within_boundaries(mission, current, target) &&
        !path_drives_over_completed(mission, report, current, target, active_line_id)) {
        return std::nullopt;
    }
    if (auto avoid_completed = avoidance_waypoint(mission, report, current, target, active_line_id)) {
        return avoid_completed;
    }
    return boundary_avoidance_waypoint(mission, report, current, target, active_line_id);
}

bool route_reachable(const sim::Mission &mission,
                     const sim::PaintReport &report,
                     sim::Point current,
                     sim::Point target,
                     const std::string &active_line_id) {
    return (sim::path_within_boundaries(mission, current, target) &&
            !path_drives_over_completed(mission, report, current, target, active_line_id)) ||
           navigation_waypoint(mission, report, current, target, active_line_id).has_value();
}

bool safe_non_paint_leg(const sim::Mission &mission,
                        const sim::PaintReport &report,
                        sim::Point current,
                        sim::Point target) {
    return sim::path_within_boundaries(mission, current, target) &&
           !path_drives_over_completed(mission, report, current, target, "");
}

std::optional<sim::Point> return_home_corridor_waypoint(const sim::Mission &mission,
                                                        const sim::PaintReport &report,
                                                        sim::Point current,
                                                        sim::Point home) {
    double max_line_y = -std::numeric_limits<double>::infinity();
    for (const auto &segment : mission.segments) {
        max_line_y = std::max(max_line_y, std::max(segment.start.y, segment.end.y));
    }
    if (!std::isfinite(max_line_y)) {
        return std::nullopt;
    }

    const std::vector<double> top_y_candidates{
        max_line_y + 0.08,
        max_line_y + mission.completion_tolerance_m * 0.8,
        max_line_y + mission.completion_tolerance_m * 1.2,
    };
    const double side_clearance = std::max(mission.avoid_clearance_m * 1.5, mission.completion_tolerance_m * 2.5);
    const std::vector<double> side_x_candidates{
        home.x - side_clearance,
        home.x + side_clearance,
    };

    for (const double top_y : top_y_candidates) {
        for (const double side_x : side_x_candidates) {
            const sim::Point lift{.x = current.x, .y = top_y};
            const sim::Point cross{.x = side_x, .y = top_y};
            const sim::Point drop{.x = side_x, .y = home.y};
            if (!sim::point_within_boundaries(mission, lift) ||
                !sim::point_within_boundaries(mission, cross) ||
                !sim::point_within_boundaries(mission, drop)) {
                continue;
            }
            if (std::abs(current.x - side_x) < 0.5 &&
                current.y <= home.y + 0.7 &&
                safe_non_paint_leg(mission, report, current, home)) {
                return home;
            }
            if (std::abs(current.x - side_x) < 0.35 &&
                sim::distance(current, drop) > 0.25 &&
                safe_non_paint_leg(mission, report, current, drop)) {
                return drop;
            }
            if (sim::distance(current, lift) > 0.25 && safe_non_paint_leg(mission, report, current, lift)) {
                return lift;
            }
            if (sim::distance(current, cross) > 0.25 && safe_non_paint_leg(mission, report, current, cross)) {
                return cross;
            }
            if (sim::distance(current, drop) > 0.25 && safe_non_paint_leg(mission, report, current, drop)) {
                return drop;
            }
            if (safe_non_paint_leg(mission, report, current, home)) {
                return home;
            }
        }
    }

    return std::nullopt;
}

LineFeasibility evaluate_line(const sim::Mission &mission,
                              const sim::PaintReport &report,
                              const sim::Segment &segment,
                              sim::Point current) {
    LineFeasibility result{
        .feasible = true,
        .center_segment = sim::robot_center_segment_for_paint_segment(segment, mission),
    };

    const sim::Point paint_start = segment.start;
    const sim::Point paint_end = segment.end;
    if (!sim::path_within_boundaries(mission, paint_start, paint_end) ||
        !sim::point_respects_boundary_buffer(mission, paint_start, mission.line_boundary_buffer_m) ||
        !sim::point_respects_boundary_buffer(mission, paint_end, mission.line_boundary_buffer_m)) {
        result.feasible = false;
        result.reason = "boundary_proximity";
        return result;
    }

    const bool entry_is_start = true;
    const sim::Point center_stage = staging_point(result.center_segment, entry_is_start, mission);
    const sim::Point center_entry = endpoint(result.center_segment, entry_is_start);
    const sim::Point center_exit = endpoint(result.center_segment, !entry_is_start);

    if (!sim::point_within_boundaries(mission, center_stage) ||
        !sim::path_within_boundaries(mission, center_stage, center_entry) ||
        !sim::path_within_boundaries(mission, center_entry, center_exit)) {
        result.feasible = false;
        result.reason = "paint_arm_unreachable";
        return result;
    }

    if (path_drives_over_completed(mission, report, center_stage, center_entry, segment.id) ||
        path_drives_over_completed(mission, report, center_entry, center_exit, segment.id)) {
        result.feasible = false;
        result.reason = "fresh_paint_crossing";
        return result;
    }

    if (!route_reachable(mission, report, current, center_stage, segment.id)) {
        result.feasible = false;
        if (path_drives_over_completed(mission, report, current, center_stage, segment.id)) {
            result.reason = "fresh_paint_crossing";
        } else {
            result.reason = "paint_arm_unreachable";
        }
        return result;
    }

    return result;
}

std::optional<SelectedTarget> select_next_line(const sim::Mission &mission,
                                               const sim::PaintReport &report,
                                               sim::Point current,
                                               std::map<std::string, std::string> &infeasible) {
    double best_score = std::numeric_limits<double>::infinity();
    std::optional<SelectedTarget> best;

    for (const auto &segment : mission.segments) {
        if (report.completed.contains(segment.id) || infeasible.contains(segment.id)) {
            continue;
        }

        const bool entry_is_start = true;
        const auto feasibility = evaluate_line(mission, report, segment, current);
        if (!feasibility.feasible) {
            if (feasibility.reason == "boundary_proximity") {
                infeasible[segment.id] = feasibility.reason;
            }
            continue;
        }
        const sim::Point stage = staging_point(feasibility.center_segment, entry_is_start, mission);

        double score = sim::distance(current, stage);
        if (blocking_completed_segment(mission, report, current, stage, segment.id)) {
            score += 4.0;
        }
        if (!sim::path_within_boundaries(mission, current, stage)) {
            score += 4.0;
        }
        if (score < best_score) {
            best_score = score;
            best = SelectedTarget{.segment = segment, .center_segment = feasibility.center_segment, .entry_is_start = entry_is_start};
        }
    }

    return best;
}

void mark_remaining_infeasible(const sim::Mission &mission,
                               const sim::PaintReport &report,
                               sim::Point current,
                               std::map<std::string, std::string> &infeasible) {
    for (const auto &segment : mission.segments) {
        if (report.completed.contains(segment.id) || infeasible.contains(segment.id)) {
            continue;
        }
        const auto feasibility = evaluate_line(mission, report, segment, current);
        infeasible[segment.id] = feasibility.feasible ? "paint_arm_unreachable" : feasibility.reason;
    }
}

size_t accounted_lines(const sim::Mission &mission,
                       const sim::PaintReport &report,
                       const std::map<std::string, std::string> &infeasible) {
    size_t count = report.completed.size();
    for (const auto &[line_id, _] : infeasible) {
        if (find_segment(mission, line_id) && !report.completed.contains(line_id)) {
            count += 1;
        }
    }
    return count;
}

bool same_mission_lines(const sim::Mission &left, const sim::Mission &right) {
    if (left.segments.size() != right.segments.size()) {
        return false;
    }
    for (size_t index = 0; index < left.segments.size(); ++index) {
        if (left.segments[index].id != right.segments[index].id) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    std::optional<sim::Mission> mission;
    std::optional<sim::RobotState> state;
    sim::PaintReport report;
    std::optional<ActiveTarget> active;
    std::map<std::string, std::string> infeasible;

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind != sim::EventKind::Input) {
            continue;
        }

        if (event->id == "mission") {
            if (auto parsed = sim::parse_mission(event->data)) {
                if (!mission || !same_mission_lines(*mission, *parsed)) {
                    active.reset();
                    infeasible.clear();
                }
                mission = *parsed;
            }
            continue;
        }
        if (event->id == "state_estimate") {
            state = sim::parse_state(event->data);
            continue;
        }
        if (event->id == "paint_report") {
            if (auto parsed = sim::parse_paint_report(event->data)) {
                report = *parsed;
            }
            continue;
        }
        if (event->id != "tick" || !mission || !state) {
            continue;
        }

        sim::Point current{.x = state->x, .y = state->y};
        sim::Plan plan;
        auto send_plan = [&]() {
            plan.infeasible = infeasible;
            node.send("plan", sim::render_plan(plan));
        };

        for (const auto &line_id : report.completed) {
            infeasible.erase(line_id);
        }

        if (active && report.completed.contains(active->line_id)) {
            active.reset();
        }
        if (active && (!find_segment(*mission, active->line_id) || infeasible.contains(active->line_id))) {
            active.reset();
        }

        if (!active) {
            if (auto selected = select_next_line(*mission, report, current, infeasible)) {
                active = ActiveTarget{
                    .line_id = selected->segment.id,
                    .entry_is_start = selected->entry_is_start,
                };
            }
        }

        const auto target_segment = active ? find_segment(*mission, active->line_id) : std::nullopt;
        if (!target_segment) {
            if (accounted_lines(*mission, report, infeasible) < mission->segments.size()) {
                mark_remaining_infeasible(*mission, report, current, infeasible);
            }

            if (accounted_lines(*mission, report, infeasible) < mission->segments.size()) {
                plan.mode = "no_reachable_lines";
                plan.target_x = current.x;
                plan.target_y = current.y;
                plan.desired_speed_mps = 0.0;
                send_plan();
                continue;
            }

            sim::Point home{.x = mission->home.x, .y = mission->home.y};
            plan.mode = sim::distance(current, home) < 0.2 ? "done" : "return_home";
            sim::Point target = home;
            if (plan.mode != "done") {
                const bool fresh_paint_blocked = path_drives_over_completed(*mission, report, current, target, "");
                if (auto corridor = return_home_corridor_waypoint(*mission, report, current, home)) {
                    target = *corridor;
                    plan.mode = "return_home";
                } else if (auto avoid = navigation_waypoint(*mission, report, current, target, "")) {
                    target = *avoid;
                    plan.mode = fresh_paint_blocked ? "avoid_completed_line" : "avoid_boundary";
                }
                if (!sim::path_within_boundaries(*mission, current, target) ||
                    path_drives_over_completed(*mission, report, current, target, "")) {
                    target = current;
                    plan.mode = "boundary_hold";
                    plan.desired_speed_mps = 0.0;
                }
            }
            plan.target_x = target.x;
            plan.target_y = target.y;
            plan.desired_speed_mps =
                (plan.mode == "done" || plan.mode == "boundary_hold") ? 0.0 : mission->max_speed_mps * 0.55;
            send_plan();
            continue;
        }

        const sim::Segment center_segment = sim::robot_center_segment_for_paint_segment(*target_segment, *mission);
        const sim::Point entry = endpoint(center_segment, active->entry_is_start);
        const sim::Point exit = endpoint(center_segment, !active->entry_is_start);
        const sim::Point paint_entry = endpoint(*target_segment, active->entry_is_start);
        const sim::Point paint_exit = endpoint(*target_segment, !active->entry_is_start);
        const sim::Point stage = staging_point(center_segment, active->entry_is_start, *mission);
        const sim::Point nozzle = paint_point_from_state(*state, *mission);
        const double stage_distance = sim::distance(current, stage);
        const double entry_distance = sim::distance(current, entry);
        const double lateral_error =
            std::abs(sim::signed_lateral_error(paint_entry, paint_exit, nozzle));
        const double along = sim::projection_ratio(paint_entry, paint_exit, nozzle);
        const bool on_line = lateral_error <= mission->completion_tolerance_m && along >= -0.03 && along <= 1.03;
        const bool staged = stage_distance < 0.35;
        const bool close_to_entry = entry_distance < 0.22;
        if (staged) {
            active->staged = true;
        }
        if (active->staged && (close_to_entry || (on_line && along >= -0.02 && along <= 0.08))) {
            active->entered = true;
        }

        sim::Point target = stage;
        plan.mode = "stage_line";
        plan.line_id = target_segment->id;
        plan.paint_enabled = false;
        plan.desired_speed_mps = mission->max_speed_mps * 0.55;

        if (active->staged) {
            target = entry;
            plan.mode = "line_entry";
            plan.desired_speed_mps = mission->max_speed_mps * 0.35;
        }

        if (active->entered) {
            target = exit;
            plan.mode = on_line ? "paint" : "line_entry";
            plan.paint_enabled = on_line && along >= -0.01 && along <= 0.99;
            plan.desired_speed_mps = plan.paint_enabled ? mission->max_speed_mps * 0.38 : mission->max_speed_mps * 0.45;
        }

        if (!plan.paint_enabled) {
            const bool fresh_paint_blocked =
                path_drives_over_completed(*mission, report, current, target, target_segment->id);
            if (auto avoid = navigation_waypoint(*mission, report, current, target, target_segment->id)) {
                target = *avoid;
                plan.mode = fresh_paint_blocked ? "avoid_completed_line" : "avoid_boundary";
                plan.desired_speed_mps = mission->max_speed_mps * 0.38;
            }
        }
        if (!sim::path_within_boundaries(*mission, current, target) ||
            (!plan.paint_enabled && path_drives_over_completed(*mission, report, current, target, target_segment->id))) {
            target = current;
            plan.mode = "boundary_hold";
            plan.paint_enabled = false;
            plan.desired_speed_mps = 0.0;
        }

        plan.target_x = target.x;
        plan.target_y = target.y;
        send_plan();
    }

    return 0;
}
