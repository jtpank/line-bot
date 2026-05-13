#include <chrono>
#include <iostream>
#include <optional>

#include "common/boundaries.hh"
#include "common/dora_node.hh"
#include "common/geometry.hh"
#include "common/messages.hh"
#include "common/paint_arm.hh"

namespace {

constexpr double kMinTrackWidthM = 0.1;

double dt_seconds(std::optional<std::chrono::steady_clock::time_point> &last, double time_scale) {
    const auto now = std::chrono::steady_clock::now();
    if (!last) {
        last = now;
        return 0.1 * time_scale;
    }
    const double dt = std::chrono::duration<double>(now - *last).count() * time_scale;
    last = now;
    return sim::clamp(dt, 0.005, 0.5);
}

}  // namespace

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    sim::Mission mission;
    sim::RobotState state;
    sim::DriveCommand drive;
    sim::PaintCommand paint;
    bool have_mission = false;
    double left_motor_mps = 0.0;
    double right_motor_mps = 0.0;
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
            if (auto parsed = sim::parse_mission(event->data)) {
                mission = *parsed;
                if (!have_mission) {
                    const sim::Pose start_pose = sim::valid_start_pose(mission);
                    state.x = start_pose.x;
                    state.y = start_pose.y;
                    state.yaw = start_pose.yaw;
                    have_mission = true;
                }
            }
            continue;
        }
        if (event->id == "drive_command") {
            if (auto parsed = sim::parse_drive_command(event->data)) {
                drive = *parsed;
            }
            continue;
        }
        if (event->id == "paint_command") {
            if (auto parsed = sim::parse_paint_command(event->data)) {
                paint = *parsed;
            }
            continue;
        }
        if (event->id != "tick" || !have_mission) {
            continue;
        }

        const double dt = dt_seconds(last_tick, mission.sim_time_scale);
        const double target_left = sim::clamp(drive.left_motor_mps, -mission.max_speed_mps, mission.max_speed_mps);
        const double target_right = sim::clamp(drive.right_motor_mps, -mission.max_speed_mps, mission.max_speed_mps);
        const double max_delta_v = mission.max_accel_mps2 * dt;
        left_motor_mps += sim::clamp(target_left - left_motor_mps, -max_delta_v, max_delta_v);
        right_motor_mps += sim::clamp(target_right - right_motor_mps, -max_delta_v, max_delta_v);

        const double track_width = std::max(mission.track_width_m, kMinTrackWidthM);
        state.speed_mps = (left_motor_mps + right_motor_mps) * 0.5;
        const double yaw_rate_rps = (right_motor_mps - left_motor_mps) / track_width;

        state.yaw = sim::wrap_angle(state.yaw + yaw_rate_rps * dt);
        const sim::Point next_position{
            .x = state.x + state.speed_mps * std::cos(state.yaw) * dt,
            .y = state.y + state.speed_mps * std::sin(state.yaw) * dt,
        };
        if (sim::point_within_boundaries(mission, next_position)) {
            state.x = next_position.x;
            state.y = next_position.y;
        } else {
            left_motor_mps = 0.0;
            right_motor_mps = 0.0;
            state.speed_mps = 0.0;
        }
        state.t += dt;

        node.send("true_state", sim::render_state(state));
        const sim::Point nozzle = sim::paint_arm_world(sim::Pose{.x = state.x, .y = state.y, .yaw = state.yaw}, mission);
        node.send("paint_event", sim::render_paint_event(sim::PaintEvent{
                                   .t = state.t,
                                   .x = nozzle.x,
                                   .y = nozzle.y,
                                   .robot_x = state.x,
                                   .robot_y = state.y,
                                   .yaw = state.yaw,
                                   .enabled = paint.enabled,
                                   .line_id = paint.line_id,
                               }));
    }

    return 0;
}
