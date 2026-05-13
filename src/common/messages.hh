#pragma once

#include <optional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/geometry.hh"

namespace sim {

struct Message {
    std::vector<std::pair<std::string, std::string>> fields;

    std::optional<std::string> get(const std::string &key) const;
    std::vector<std::string> get_all(const std::string &key) const;
};

struct Segment {
    std::string id;
    Point start;
    Point end;
};

struct Boundary {
    std::string id;
    std::string kind{"keep_in"};
    std::vector<Point> vertices;
};

struct GpsPoint {
    double lat{0.0};
    double lon{0.0};
    double alt{0.0};
};

struct Mission {
    std::string units{"meters"};
    Pose home;
    std::optional<GpsPoint> gps_origin;
    double track_width_m{0.75};
    double max_speed_mps{0.8};
    double max_accel_mps2{0.8};
    double paint_width_m{0.08};
    double completion_tolerance_m{0.15};
    double avoid_clearance_m{0.45};
    double line_boundary_buffer_m{0.0};
    double paint_arm_forward_m{0.0};
    double paint_arm_left_m{0.0};
    double paint_arm_z_m{0.0};
    double sim_time_scale{1.0};
    std::vector<Segment> segments;
    std::vector<Boundary> boundaries;
};

struct RobotState {
    double t{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed_mps{0.0};
};

struct DriveCommand {
    double left_motor_mps{0.0};
    double right_motor_mps{0.0};
};

struct PaintCommand {
    bool enabled{false};
    std::string line_id;
};

struct PaintEvent {
    double t{0.0};
    double x{0.0};
    double y{0.0};
    double robot_x{0.0};
    double robot_y{0.0};
    double yaw{0.0};
    bool enabled{false};
    std::string line_id;
};

struct PaintReport {
    std::set<std::string> completed;
    int drive_over_violations{0};
    bool all_complete{false};
};

struct Plan {
    std::string mode{"idle"};
    double target_x{0.0};
    double target_y{0.0};
    std::string line_id;
    bool entry_is_start{true};
    bool paint_enabled{false};
    double desired_speed_mps{0.0};
    std::map<std::string, std::string> infeasible;
};

struct GpsFix {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    std::string fix{"rtk"};
};

struct ImuSample {
    double yaw{0.0};
    double yaw_rate_rps{0.0};
    double accel_mps2{0.0};
};

Message parse_message(const std::string &data);
std::string render_message(const std::vector<std::pair<std::string, std::string>> &fields);
std::vector<std::string> split(const std::string &value, char delimiter);

std::optional<Mission> parse_mission(const std::string &data);
std::string render_mission(const Mission &mission);

std::optional<RobotState> parse_state(const std::string &data);
std::string render_state(const RobotState &state);

std::optional<DriveCommand> parse_drive_command(const std::string &data);
std::string render_drive_command(const DriveCommand &command);

std::optional<PaintCommand> parse_paint_command(const std::string &data);
std::string render_paint_command(const PaintCommand &command);

std::optional<PaintEvent> parse_paint_event(const std::string &data);
std::string render_paint_event(const PaintEvent &event);

std::optional<PaintReport> parse_paint_report(const std::string &data);
std::string render_paint_report(const PaintReport &report);

std::optional<Plan> parse_plan(const std::string &data);
std::string render_plan(const Plan &plan);

std::optional<GpsFix> parse_gps_fix(const std::string &data);
std::string render_gps_fix(const GpsFix &fix);

std::optional<ImuSample> parse_imu(const std::string &data);
std::string render_imu(const ImuSample &imu);

bool parse_bool(const std::string &value);
std::string bool_text(bool value);
double parse_double_or(const std::optional<std::string> &value, double fallback);

}  // namespace sim
