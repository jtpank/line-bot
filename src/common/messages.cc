#include "common/messages.hh"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace sim {
namespace {

constexpr double kDegreesToRadians = kPi / 180.0;

std::optional<double> parse_double(const std::string &value) {
    char *end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value.c_str(), &end);
    if (errno != 0 || end == value.c_str() || *end != '\0') {
        return std::nullopt;
    }
    return parsed;
}

std::string number(double value) {
    std::ostringstream out;
    out.precision(12);
    out << value;
    return out.str();
}

std::string join(const std::set<std::string> &values) {
    std::string out;
    bool first = true;
    for (const auto &value : values) {
        if (!first) {
            out += ",";
        }
        out += value;
        first = false;
    }
    return out;
}

std::string join(const std::map<std::string, std::string> &values) {
    std::string out;
    bool first = true;
    for (const auto &[line_id, reason] : values) {
        if (!first) {
            out += ";";
        }
        out += line_id;
        out += ":";
        out += reason;
        first = false;
    }
    return out;
}

std::map<std::string, std::string> parse_reason_map(const std::string &value) {
    std::map<std::string, std::string> out;
    for (const auto &entry : split(value, ';')) {
        if (entry.empty()) {
            continue;
        }
        const auto separator = entry.find(':');
        if (separator == std::string::npos || separator == 0 || separator + 1 >= entry.size()) {
            continue;
        }
        out[entry.substr(0, separator)] = entry.substr(separator + 1);
    }
    return out;
}

std::string lowercase(std::string value) {
    for (auto &character : value) {
        if (character >= 'A' && character <= 'Z') {
            character = static_cast<char>(character - 'A' + 'a');
        }
    }
    return value;
}

double unit_scale_to_meters(const std::string &units) {
    const std::string normalized = lowercase(units);
    if (normalized == "feet" || normalized == "foot" || normalized == "ft") {
        return 0.3048;
    }
    return 1.0;
}

bool is_boundary_kind(const std::string &value) {
    const std::string normalized = lowercase(value);
    return normalized == "keep_in" || normalized == "keep-out" || normalized == "keep_out" ||
           normalized == "exclude" || normalized == "obstacle";
}

std::string normalize_boundary_kind(const std::string &value) {
    const std::string normalized = lowercase(value);
    if (normalized == "keep-out" || normalized == "keep_out" || normalized == "exclude" || normalized == "obstacle") {
        return "keep_out";
    }
    return "keep_in";
}

double parse_distance_or(const Message &message,
                         const std::string &configured_units_key,
                         const std::string &meters_key,
                         double fallback_m,
                         double unit_scale_m) {
    if (const auto explicit_meters = message.get(meters_key)) {
        return parse_double(*explicit_meters).value_or(fallback_m);
    }
    if (const auto configured_units = message.get(configured_units_key)) {
        return parse_double(*configured_units).value_or(fallback_m / unit_scale_m) * unit_scale_m;
    }
    return fallback_m;
}

void parse_offset(const Message &message,
                  const std::string &key,
                  double unit_scale_m,
                  double &forward_m,
                  double &left_m,
                  double &z_m) {
    if (const auto offset_text = message.get(key)) {
        const auto parts = split(*offset_text, ',');
        if (parts.size() >= 2) {
            if (const auto forward = parse_double(parts[0])) {
                forward_m = *forward * unit_scale_m;
            }
            if (const auto left = parse_double(parts[1])) {
                left_m = *left * unit_scale_m;
            }
            if (parts.size() >= 3) {
                if (const auto z = parse_double(parts[2])) {
                    z_m = *z * unit_scale_m;
                }
            }
        }
    }
}

std::optional<Point> parse_point_pair(const std::vector<std::string> &parts, size_t offset) {
    if (parts.size() < offset + 2) {
        return std::nullopt;
    }
    auto x = parse_double(parts[offset]);
    auto y = parse_double(parts[offset + 1]);
    if (!x || !y) {
        return std::nullopt;
    }
    return Point{.x = *x, .y = *y};
}

std::optional<GpsPoint> parse_gps_point(const std::vector<std::string> &parts, size_t offset) {
    if (parts.size() < offset + 3) {
        return std::nullopt;
    }
    auto lat = parse_double(parts[offset]);
    auto lon = parse_double(parts[offset + 1]);
    auto alt = parse_double(parts[offset + 2]);
    if (!lat || !lon || !alt) {
        return std::nullopt;
    }
    return GpsPoint{.lat = *lat, .lon = *lon, .alt = *alt};
}

Point gps_to_local_meters(GpsPoint point, GpsPoint origin) {
    const double origin_lat_rad = origin.lat * kDegreesToRadians;
    const double meters_per_degree_lat = 111132.92 - 559.82 * std::cos(2.0 * origin_lat_rad) +
                                         1.175 * std::cos(4.0 * origin_lat_rad) -
                                         0.0023 * std::cos(6.0 * origin_lat_rad);
    const double meters_per_degree_lon = 111412.84 * std::cos(origin_lat_rad) -
                                         93.5 * std::cos(3.0 * origin_lat_rad) +
                                         0.118 * std::cos(5.0 * origin_lat_rad);
    return Point{
        .x = (point.lon - origin.lon) * meters_per_degree_lon,
        .y = (point.lat - origin.lat) * meters_per_degree_lat,
    };
}

std::optional<Boundary> parse_local_boundary(const std::string &boundary_text) {
    const auto id_and_vertices = split(boundary_text, ',');
    if (id_and_vertices.size() < 2) {
        return std::nullopt;
    }

    Boundary boundary{.id = id_and_vertices[0]};
    std::string vertices_text = boundary_text.substr(boundary.id.size() + 1);
    const auto maybe_kind = split(vertices_text, ',');
    if (!maybe_kind.empty() && is_boundary_kind(maybe_kind[0])) {
        boundary.kind = normalize_boundary_kind(maybe_kind[0]);
        vertices_text = vertices_text.substr(maybe_kind[0].size() + 1);
    }

    const auto vertex_texts = split(vertices_text, '|');
    for (const auto &vertex_text : vertex_texts) {
        const auto parts = split(vertex_text, ',');
        const auto point = parse_point_pair(parts, 0);
        if (!point) {
            return std::nullopt;
        }
        boundary.vertices.push_back(*point);
    }
    return boundary.vertices.size() >= 3 ? std::optional<Boundary>(boundary) : std::nullopt;
}

std::optional<Boundary> parse_gps_boundary(const std::string &boundary_text, GpsPoint origin) {
    const auto id_and_vertices = split(boundary_text, ',');
    if (id_and_vertices.size() < 2) {
        return std::nullopt;
    }

    Boundary boundary{.id = id_and_vertices[0]};
    std::string vertices_text = boundary_text.substr(boundary.id.size() + 1);
    const auto maybe_kind = split(vertices_text, ',');
    if (!maybe_kind.empty() && is_boundary_kind(maybe_kind[0])) {
        boundary.kind = normalize_boundary_kind(maybe_kind[0]);
        vertices_text = vertices_text.substr(maybe_kind[0].size() + 1);
    }

    const auto vertex_texts = split(vertices_text, '|');
    for (const auto &vertex_text : vertex_texts) {
        const auto parts = split(vertex_text, ',');
        const auto point = parse_gps_point(parts, 0);
        if (!point) {
            return std::nullopt;
        }
        boundary.vertices.push_back(gps_to_local_meters(*point, origin));
    }
    return boundary.vertices.size() >= 3 ? std::optional<Boundary>(boundary) : std::nullopt;
}

}  // namespace

std::optional<std::string> Message::get(const std::string &key) const {
    for (auto it = fields.rbegin(); it != fields.rend(); ++it) {
        if (it->first == key) {
            return it->second;
        }
    }
    return std::nullopt;
}

std::vector<std::string> Message::get_all(const std::string &key) const {
    std::vector<std::string> values;
    for (const auto &[field_key, value] : fields) {
        if (field_key == key) {
            values.push_back(value);
        }
    }
    return values;
}

Message parse_message(const std::string &data) {
    Message message;
    std::istringstream input(data);
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            continue;
        }
        message.fields.emplace_back(line.substr(0, equals), line.substr(equals + 1));
    }
    return message;
}

std::string render_message(const std::vector<std::pair<std::string, std::string>> &fields) {
    std::string out;
    for (const auto &[key, value] : fields) {
        out += key;
        out += "=";
        out += value;
        out += "\n";
    }
    return out;
}

std::vector<std::string> split(const std::string &value, char delimiter) {
    std::vector<std::string> parts;
    std::string current;
    std::istringstream input(value);
    while (std::getline(input, current, delimiter)) {
        parts.push_back(current);
    }
    return parts;
}

bool parse_bool(const std::string &value) {
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

std::string bool_text(bool value) {
    return value ? "1" : "0";
}

double parse_double_or(const std::optional<std::string> &value, double fallback) {
    if (!value) {
        return fallback;
    }
    return parse_double(*value).value_or(fallback);
}

std::optional<Mission> parse_mission(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "mission") {
        return std::nullopt;
    }

    Mission mission;
    mission.units = lowercase(message.get("units").value_or(mission.units));
    const double unit_scale_m = unit_scale_to_meters(mission.units);

    if (const auto origin_text = message.get("gps_origin")) {
        const auto parts = split(*origin_text, ',');
        mission.gps_origin = parse_gps_point(parts, 0);
    }

    const auto home_parts = split(message.get("home").value_or("0,0,0"), ',');
    if (home_parts.size() >= 3) {
        auto x = parse_double(home_parts[0]);
        auto y = parse_double(home_parts[1]);
        auto yaw = parse_double(home_parts[2]);
        if (x && y && yaw) {
            mission.home = Pose{.x = *x, .y = *y, .yaw = *yaw};
        }
    }
    if (const auto home_gps_text = message.get("home_gps"); home_gps_text && mission.gps_origin) {
        const auto parts = split(*home_gps_text, ',');
        const auto point = parse_gps_point(parts, 0);
        const auto yaw = parts.size() >= 4 ? parse_double(parts[3]) : std::optional<double>{0.0};
        if (point && yaw) {
            const auto local = gps_to_local_meters(*point, *mission.gps_origin);
            mission.home = Pose{.x = local.x, .y = local.y, .yaw = *yaw};
        }
    }

    mission.track_width_m = parse_distance_or(message, "track_width", "track_width_m", mission.track_width_m, unit_scale_m);
    mission.max_speed_mps = parse_distance_or(message, "max_speed", "max_speed_mps", mission.max_speed_mps, unit_scale_m);
    mission.max_accel_mps2 = parse_distance_or(message, "max_accel", "max_accel_mps2", mission.max_accel_mps2, unit_scale_m);
    mission.paint_width_m = parse_distance_or(message, "paint_width", "paint_width_m", mission.paint_width_m, unit_scale_m);
    mission.completion_tolerance_m =
        parse_distance_or(message, "completion_tolerance", "completion_tolerance_m", mission.completion_tolerance_m, unit_scale_m);
    mission.avoid_clearance_m =
        parse_distance_or(message, "avoid_clearance", "avoid_clearance_m", mission.avoid_clearance_m, unit_scale_m);
    mission.line_boundary_buffer_m =
        parse_distance_or(message, "line_boundary_buffer", "line_boundary_buffer_m", mission.line_boundary_buffer_m, unit_scale_m);
    parse_offset(message,
                 "paint_arm_offset_m",
                 1.0,
                 mission.paint_arm_forward_m,
                 mission.paint_arm_left_m,
                 mission.paint_arm_z_m);
    parse_offset(message,
                 "paint_arm_offset",
                 unit_scale_m,
                 mission.paint_arm_forward_m,
                 mission.paint_arm_left_m,
                 mission.paint_arm_z_m);
    mission.paint_arm_forward_m =
        parse_distance_or(message, "paint_arm_forward", "paint_arm_forward_m", mission.paint_arm_forward_m, unit_scale_m);
    mission.paint_arm_left_m =
        parse_distance_or(message, "paint_arm_left", "paint_arm_left_m", mission.paint_arm_left_m, unit_scale_m);
    mission.paint_arm_z_m =
        parse_distance_or(message, "paint_arm_z", "paint_arm_z_m", mission.paint_arm_z_m, unit_scale_m);
    mission.sim_time_scale = std::max(0.1, parse_double_or(message.get("sim_time_scale"), mission.sim_time_scale));

    for (const auto &segment_text : message.get_all("segment")) {
        const auto parts = split(segment_text, ',');
        if (parts.size() < 5) {
            continue;
        }
        const auto start = parse_point_pair(parts, 1);
        const auto end = parse_point_pair(parts, 3);
        if (!start || !end) {
            continue;
        }
        mission.segments.push_back(Segment{.id = parts[0], .start = *start, .end = *end});
    }
    if (mission.gps_origin) {
        for (const auto &segment_text : message.get_all("segment_gps")) {
            const auto parts = split(segment_text, ',');
            if (parts.size() < 6) {
                continue;
            }
            const auto start_gps = parse_gps_point(parts, 1);
            const auto bearing_degrees = parse_double(parts[4]);
            const auto length_units = parse_double(parts[5]);
            if (!start_gps || !bearing_degrees || !length_units) {
                continue;
            }
            const double length_m = *length_units * unit_scale_m;
            const double bearing_rad = *bearing_degrees * kDegreesToRadians;
            const Point start = gps_to_local_meters(*start_gps, *mission.gps_origin);
            const Point end{
                .x = start.x + std::sin(bearing_rad) * length_m,
                .y = start.y + std::cos(bearing_rad) * length_m,
            };
            mission.segments.push_back(Segment{.id = parts[0], .start = start, .end = end});
        }
    }
    for (const auto &boundary_text : message.get_all("boundary")) {
        if (auto boundary = parse_local_boundary(boundary_text)) {
            mission.boundaries.push_back(*boundary);
        }
    }
    if (mission.gps_origin) {
        for (const auto &boundary_text : message.get_all("boundary_gps")) {
            if (auto boundary = parse_gps_boundary(boundary_text, *mission.gps_origin)) {
                mission.boundaries.push_back(*boundary);
            }
        }
    }
    return mission;
}

std::string render_mission(const Mission &mission) {
    std::vector<std::pair<std::string, std::string>> fields{
        {"type", "mission"},
        {"units", mission.units},
        {"home", number(mission.home.x) + "," + number(mission.home.y) + "," + number(mission.home.yaw)},
        {"track_width_m", number(mission.track_width_m)},
        {"max_speed_mps", number(mission.max_speed_mps)},
        {"max_accel_mps2", number(mission.max_accel_mps2)},
        {"paint_width_m", number(mission.paint_width_m)},
        {"completion_tolerance_m", number(mission.completion_tolerance_m)},
        {"avoid_clearance_m", number(mission.avoid_clearance_m)},
        {"line_boundary_buffer_m", number(mission.line_boundary_buffer_m)},
        {"paint_arm_forward_m", number(mission.paint_arm_forward_m)},
        {"paint_arm_left_m", number(mission.paint_arm_left_m)},
        {"paint_arm_z_m", number(mission.paint_arm_z_m)},
        {"sim_time_scale", number(mission.sim_time_scale)},
    };
    for (const auto &segment : mission.segments) {
        fields.push_back({"segment",
                          segment.id + "," + number(segment.start.x) + "," + number(segment.start.y) + "," +
                              number(segment.end.x) + "," + number(segment.end.y)});
    }
    for (const auto &boundary : mission.boundaries) {
        std::string value = boundary.id + "," + boundary.kind + ",";
        bool first = true;
        for (const auto &vertex : boundary.vertices) {
            if (!first) {
                value += "|";
            }
            value += number(vertex.x) + "," + number(vertex.y);
            first = false;
        }
        fields.push_back({"boundary", value});
    }
    return render_message(fields);
}

std::optional<RobotState> parse_state(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "state") {
        return std::nullopt;
    }
    RobotState state;
    state.t = parse_double_or(message.get("t"), state.t);
    state.x = parse_double_or(message.get("x"), state.x);
    state.y = parse_double_or(message.get("y"), state.y);
    state.yaw = parse_double_or(message.get("yaw"), state.yaw);
    state.speed_mps = parse_double_or(message.get("speed_mps"), state.speed_mps);
    return state;
}

std::string render_state(const RobotState &state) {
    return render_message({
        {"type", "state"},
        {"t", number(state.t)},
        {"x", number(state.x)},
        {"y", number(state.y)},
        {"yaw", number(state.yaw)},
        {"speed_mps", number(state.speed_mps)},
    });
}

std::optional<DriveCommand> parse_drive_command(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "drive_command") {
        return std::nullopt;
    }
    DriveCommand command;
    command.left_motor_mps = parse_double_or(message.get("left_motor_mps"), command.left_motor_mps);
    command.right_motor_mps = parse_double_or(message.get("right_motor_mps"), command.right_motor_mps);
    return command;
}

std::string render_drive_command(const DriveCommand &command) {
    return render_message({
        {"type", "drive_command"},
        {"left_motor_mps", number(command.left_motor_mps)},
        {"right_motor_mps", number(command.right_motor_mps)},
    });
}

std::optional<PaintCommand> parse_paint_command(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "paint_command") {
        return std::nullopt;
    }
    PaintCommand command;
    command.enabled = parse_bool(message.get("enabled").value_or("0"));
    command.line_id = message.get("line_id").value_or("");
    return command;
}

std::string render_paint_command(const PaintCommand &command) {
    return render_message({
        {"type", "paint_command"},
        {"enabled", bool_text(command.enabled)},
        {"line_id", command.line_id},
    });
}

std::optional<PaintEvent> parse_paint_event(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "paint_event") {
        return std::nullopt;
    }
    PaintEvent event;
    event.t = parse_double_or(message.get("t"), event.t);
    event.x = parse_double_or(message.get("x"), event.x);
    event.y = parse_double_or(message.get("y"), event.y);
    event.robot_x = parse_double_or(message.get("robot_x"), event.x);
    event.robot_y = parse_double_or(message.get("robot_y"), event.y);
    event.yaw = parse_double_or(message.get("yaw"), event.yaw);
    event.enabled = parse_bool(message.get("enabled").value_or("0"));
    event.line_id = message.get("line_id").value_or("");
    return event;
}

std::string render_paint_event(const PaintEvent &event) {
    return render_message({
        {"type", "paint_event"},
        {"t", number(event.t)},
        {"x", number(event.x)},
        {"y", number(event.y)},
        {"robot_x", number(event.robot_x)},
        {"robot_y", number(event.robot_y)},
        {"yaw", number(event.yaw)},
        {"enabled", bool_text(event.enabled)},
        {"line_id", event.line_id},
    });
}

std::optional<PaintReport> parse_paint_report(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "paint_report") {
        return std::nullopt;
    }
    PaintReport report;
    for (const auto &id : split(message.get("completed").value_or(""), ',')) {
        if (!id.empty()) {
            report.completed.insert(id);
        }
    }
    report.drive_over_violations = static_cast<int>(parse_double_or(message.get("drive_over_violations"), 0.0));
    report.all_complete = parse_bool(message.get("all_complete").value_or("0"));
    return report;
}

std::string render_paint_report(const PaintReport &report) {
    return render_message({
        {"type", "paint_report"},
        {"completed", join(report.completed)},
        {"drive_over_violations", std::to_string(report.drive_over_violations)},
        {"all_complete", bool_text(report.all_complete)},
    });
}

std::optional<Plan> parse_plan(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "plan") {
        return std::nullopt;
    }
    Plan plan;
    plan.mode = message.get("mode").value_or(plan.mode);
    plan.target_x = parse_double_or(message.get("target_x"), plan.target_x);
    plan.target_y = parse_double_or(message.get("target_y"), plan.target_y);
    plan.line_id = message.get("line_id").value_or(plan.line_id);
    plan.entry_is_start = parse_bool(message.get("entry_is_start").value_or("1"));
    plan.paint_enabled = parse_bool(message.get("paint_enabled").value_or("0"));
    plan.desired_speed_mps = parse_double_or(message.get("desired_speed_mps"), plan.desired_speed_mps);
    plan.infeasible = parse_reason_map(message.get("infeasible").value_or(""));
    return plan;
}

std::string render_plan(const Plan &plan) {
    return render_message({
        {"type", "plan"},
        {"mode", plan.mode},
        {"target_x", number(plan.target_x)},
        {"target_y", number(plan.target_y)},
        {"line_id", plan.line_id},
        {"entry_is_start", bool_text(plan.entry_is_start)},
        {"paint_enabled", bool_text(plan.paint_enabled)},
        {"desired_speed_mps", number(plan.desired_speed_mps)},
        {"infeasible", join(plan.infeasible)},
    });
}

std::optional<GpsFix> parse_gps_fix(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "gps_fix") {
        return std::nullopt;
    }
    GpsFix fix;
    fix.x = parse_double_or(message.get("x"), fix.x);
    fix.y = parse_double_or(message.get("y"), fix.y);
    fix.yaw = parse_double_or(message.get("yaw"), fix.yaw);
    fix.fix = message.get("fix").value_or(fix.fix);
    return fix;
}

std::string render_gps_fix(const GpsFix &fix) {
    return render_message({
        {"type", "gps_fix"},
        {"x", number(fix.x)},
        {"y", number(fix.y)},
        {"yaw", number(fix.yaw)},
        {"fix", fix.fix},
    });
}

std::optional<ImuSample> parse_imu(const std::string &data) {
    const Message message = parse_message(data);
    if (message.get("type").value_or("") != "imu") {
        return std::nullopt;
    }
    ImuSample imu;
    imu.yaw = parse_double_or(message.get("yaw"), imu.yaw);
    imu.yaw_rate_rps = parse_double_or(message.get("yaw_rate_rps"), imu.yaw_rate_rps);
    imu.accel_mps2 = parse_double_or(message.get("accel_mps2"), imu.accel_mps2);
    return imu;
}

std::string render_imu(const ImuSample &imu) {
    return render_message({
        {"type", "imu"},
        {"yaw", number(imu.yaw)},
        {"yaw_rate_rps", number(imu.yaw_rate_rps)},
        {"accel_mps2", number(imu.accel_mps2)},
    });
}

}  // namespace sim
