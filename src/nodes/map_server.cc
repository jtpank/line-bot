#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#include "common/dora_node.hh"
#include "common/messages.hh"

namespace {

std::string env_or(const char *name, const std::string &fallback) {
    const char *value = std::getenv(name);
    return value == nullptr ? fallback : std::string(value);
}

std::optional<sim::Mission> load_mission(const std::string &path) {
    std::ifstream input(path);
    if (!input) {
        std::cerr << "failed to open mission map: " << path << "\n";
        return std::nullopt;
    }
    std::string data((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    return sim::parse_mission(data);
}

}  // namespace

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    const std::string map_path = env_or("MAP_PATH", "maps/demo_track.map");
    auto mission = load_mission(map_path);
    if (!mission || mission->segments.empty()) {
        std::cerr << "mission map has no valid segments: " << map_path << "\n";
        return 1;
    }
    if (const auto sim_time_scale = env_or("SIM_TIME_SCALE", ""); !sim_time_scale.empty()) {
        mission->sim_time_scale =
            std::max(0.1, sim::parse_double_or(std::optional<std::string>{sim_time_scale}, mission->sim_time_scale));
    }

    const std::string mission_message = sim::render_mission(*mission);
    std::cout << "loaded mission map '" << map_path << "' with " << mission->segments.size() << " segments" << std::endl;

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind == sim::EventKind::Input && event->id == "tick") {
            node.send("mission", mission_message);
        }
    }
    return 0;
}
