#include <iostream>
#include <map>
#include <optional>
#include <sstream>

#include "common/dora_node.hh"
#include "common/geometry.hh"
#include "common/messages.hh"

namespace {

std::string reason_summary(const std::map<std::string, std::string> &infeasible) {
    if (infeasible.empty()) {
        return "-";
    }
    std::map<std::string, int> counts;
    for (const auto &[_, reason] : infeasible) {
        counts[reason] += 1;
    }
    std::ostringstream out;
    bool first = true;
    for (const auto &[reason, count] : counts) {
        if (!first) {
            out << ",";
        }
        out << reason << ":" << count;
        first = false;
    }
    return out.str();
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
    sim::PaintReport report;
    int ticks = 0;
    bool announced_done = false;

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind != sim::EventKind::Input) {
            continue;
        }

        if (event->id == "mission_done") {
            break;
        } else if (event->id == "mission") {
            mission = sim::parse_mission(event->data);
        } else if (event->id == "state_estimate") {
            state = sim::parse_state(event->data);
        } else if (event->id == "plan") {
            plan = sim::parse_plan(event->data);
        } else if (event->id == "paint_report") {
            if (auto parsed = sim::parse_paint_report(event->data)) {
                report = *parsed;
            }
        } else if (event->id == "tick") {
            ticks += 1;
            if (ticks % 10 == 0 && mission && state && plan) {
                std::cout << "mode=" << plan->mode
                          << " line=" << (plan->line_id.empty() ? "-" : plan->line_id)
                          << " completed=" << report.completed.size() << "/" << mission->segments.size()
                          << " infeasible=" << plan->infeasible.size()
                          << " reasons=" << reason_summary(plan->infeasible)
                          << " violations=" << report.drive_over_violations
                          << " pose=(" << state->x << "," << state->y << "," << state->yaw << ")" << std::endl;
            }
            if (!announced_done && mission && state && plan && plan->mode == "done") {
                announced_done = true;
                std::cout << "mission complete, robot returned home with "
                          << report.drive_over_violations << " drive-over violation samples and "
                          << plan->infeasible.size() << " infeasible lines" << std::endl;
            }
        }
    }
    return 0;
}
