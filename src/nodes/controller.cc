#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "common/dora_node.hh"
#include "common/geometry.hh"
#include "common/messages.hh"
#include "common/paint_arm.hh"

namespace {

double dt_seconds(std::optional<std::chrono::steady_clock::time_point> &last, double time_scale) {
    const auto now = std::chrono::steady_clock::now();
    if (!last) {
        last = now;
        return sim::clamp(0.1 * time_scale, 0.005, 0.5);
    }
    const double dt = std::chrono::duration<double>(now - *last).count() * time_scale;
    last = now;
    return sim::clamp(dt, 0.005, 0.5);
}

double heading_turn_rate(double heading_error,
                         double dt,
                         double max_turn_rate_rps,
                         double max_turn_accel_rps2) {
    const double error = std::abs(heading_error);
    if (error < 1e-4) {
        return 0.0;
    }

    const double proportional_rate = 2.0 * error;
    const double braking_rate = std::sqrt(std::max(0.0, 2.0 * max_turn_accel_rps2 * error)) * 0.55;
    const double tick_rate = std::max(0.10, error * 0.65 / std::max(dt, 0.005));
    const double rate_limit = std::min({max_turn_rate_rps * 0.55, braking_rate, tick_rate});
    return std::copysign(std::min(proportional_rate, rate_limit), heading_error);
}

double speed_for_remaining(double remaining_m, double desired_mps, double max_accel_mps2, double dt) {
    if (remaining_m <= 0.0 || desired_mps <= 0.0) {
        return 0.0;
    }

    const double braking_speed = std::sqrt(std::max(0.0, 2.0 * max_accel_mps2 * remaining_m)) * 0.75;
    const double tick_speed = remaining_m / std::max(dt * 1.4, 0.02);
    double speed = std::min({desired_mps, braking_speed, tick_speed});
    if (remaining_m > 0.18) {
        speed = std::max(speed, std::min(0.12, desired_mps));
    }
    return speed;
}

std::optional<sim::Segment> find_segment(const sim::Mission &mission, const std::string &line_id) {
    for (const auto &segment : mission.segments) {
        if (segment.id == line_id) {
            return segment;
        }
    }
    return std::nullopt;
}

sim::Segment oriented_segment(const sim::Segment &segment, bool entry_is_start) {
    if (entry_is_start) {
        return segment;
    }
    return sim::Segment{.id = segment.id, .start = segment.end, .end = segment.start};
}

}  // namespace

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    std::optional<sim::Mission> mission;
    std::optional<sim::RobotState> state;
    std::optional<sim::Plan> plan;
    std::optional<std::chrono::steady_clock::time_point> last_tick;

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind != sim::EventKind::Input) {
            continue;
        }

        if (event->id == "mission") {
            mission = sim::parse_mission(event->data);
            continue;
        }
        if (event->id == "mission_done") {
            break;
        }
        if (event->id == "state_estimate") {
            state = sim::parse_state(event->data);
            continue;
        }
        if (event->id == "plan") {
            plan = sim::parse_plan(event->data);
            continue;
        }
        if (event->id != "tick" || !mission || !state || !plan) {
            continue;
        }

        const sim::Point current{.x = state->x, .y = state->y};
        const sim::Point target{.x = plan->target_x, .y = plan->target_y};
        const double dt = dt_seconds(last_tick, mission->sim_time_scale);

        const double max_speed = mission->max_speed_mps;
        const double track_width = std::max(mission->track_width_m, 0.1);
        const double max_turn_rate_rps = (2.0 * max_speed) / track_width;
        const double max_turn_accel_rps2 = (2.0 * mission->max_accel_mps2) / track_width;

        double linear_mps = 0.0;
        double turn_rate_rps = 0.0;
        bool valve_enabled = false;

        const auto line_segment = plan->line_id.empty() ? std::optional<sim::Segment>{}
                                                        : find_segment(*mission, plan->line_id);
        const bool line_mode = line_segment && (plan->mode == "line_entry" || plan->mode == "paint");

        if (plan->mode == "done" || plan->desired_speed_mps <= 0.0) {
            linear_mps = 0.0;
            turn_rate_rps = 0.0;
        } else if (line_mode) {
            const sim::Segment paint_segment = oriented_segment(*line_segment, plan->entry_is_start);
            const sim::Segment center_segment = sim::robot_center_segment_for_paint_segment(paint_segment, *mission);
            const double path_length = std::max(sim::segment_length(center_segment.start, center_segment.end), 1e-6);
            const double ux = (center_segment.end.x - center_segment.start.x) / path_length;
            const double uy = (center_segment.end.y - center_segment.start.y) / path_length;
            const double path_heading = std::atan2(uy, ux);
            const double lateral_error = sim::signed_lateral_error(center_segment.start, center_segment.end, current);
            const double path_heading_error = sim::wrap_angle(path_heading - state->yaw);
            const double lateral_heading_correction =
                std::abs(path_heading_error) < 0.12
                    ? sim::clamp(std::atan2(-1.1 * lateral_error, std::max(plan->desired_speed_mps, 0.25)), -0.22, 0.22)
                    : 0.0;
            const double target_heading = path_heading + lateral_heading_correction;
            const double heading_error = sim::wrap_angle(target_heading - state->yaw);
            const double remaining_along = (target.x - current.x) * ux + (target.y - current.y) * uy;

            turn_rate_rps = heading_turn_rate(heading_error, dt, max_turn_rate_rps, max_turn_accel_rps2);
            linear_mps =
                speed_for_remaining(remaining_along, plan->desired_speed_mps, mission->max_accel_mps2, dt);

            const bool aligned = std::abs(path_heading_error) < 0.12;
            const bool close_to_path = std::abs(lateral_error) < std::max(0.35, mission->completion_tolerance_m * 1.5);
            if (!aligned || !close_to_path) {
                linear_mps = 0.0;
            }
            if (remaining_along <= 0.01) {
                linear_mps = 0.0;
                turn_rate_rps = 0.0;
            }

            valve_enabled = plan->paint_enabled && linear_mps > 0.02 &&
                            std::abs(path_heading_error) < 0.25 &&
                            std::abs(lateral_error) < mission->completion_tolerance_m * 1.8;
        } else {
            const double target_distance = sim::distance(current, target);
            const double target_heading = sim::heading_to(current, target);
            const double heading_error = sim::wrap_angle(target_heading - state->yaw);
            const double distance_limited_speed =
                speed_for_remaining(target_distance, plan->desired_speed_mps, mission->max_accel_mps2, dt);
            linear_mps = distance_limited_speed;
            turn_rate_rps = heading_turn_rate(heading_error, dt, max_turn_rate_rps, max_turn_accel_rps2);
            if (std::abs(heading_error) > 0.35) {
                linear_mps = 0.0;
            } else {
                const double heading_scale = sim::clamp(1.0 - std::abs(heading_error) / 0.35, 0.35, 1.0);
                linear_mps *= heading_scale;
            }
            if (target_distance < 0.14) {
                linear_mps = 0.0;
            }
            if (target_distance < 0.06) {
                turn_rate_rps = 0.0;
            }
        }

        double left_motor_mps = linear_mps - turn_rate_rps * track_width * 0.5;
        double right_motor_mps = linear_mps + turn_rate_rps * track_width * 0.5;
        const double peak_motor_mps = std::max(std::abs(left_motor_mps), std::abs(right_motor_mps));
        if (peak_motor_mps > max_speed) {
            const double scale = max_speed / peak_motor_mps;
            left_motor_mps *= scale;
            right_motor_mps *= scale;
        }

        sim::DriveCommand drive{
            .left_motor_mps = left_motor_mps,
            .right_motor_mps = right_motor_mps,
        };
        sim::PaintCommand paint{
            .enabled = valve_enabled,
            .line_id = plan->line_id,
        };

        node.send("drive_command", sim::render_drive_command(drive));
        node.send("paint_command", sim::render_paint_command(paint));
    }
    return 0;
}
