#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>

#include "common/dora_node.hh"
#include "common/geometry.hh"
#include "common/messages.hh"

namespace {

struct Coverage {
    bool seen{false};
    double min_ratio{1.0};
    double max_ratio{0.0};
};

struct LastPaintSample {
    bool valid{false};
    std::string line_id;
    sim::Point nozzle;
};

bool update_coverage(const sim::Mission &mission,
                     const sim::Segment &segment,
                     sim::Point nozzle,
                     Coverage &coverage) {
    const double lateral = std::abs(sim::signed_lateral_error(segment.start, segment.end, nozzle));
    const double ratio = sim::projection_ratio(segment.start, segment.end, nozzle);
    const bool near_segment = lateral <= mission.completion_tolerance_m && ratio >= -0.08 && ratio <= 1.08;
    if (!near_segment) {
        return false;
    }

    const double clamped_ratio = sim::clamp(ratio, 0.0, 1.0);
    coverage.seen = true;
    coverage.min_ratio = std::min(coverage.min_ratio, clamped_ratio);
    coverage.max_ratio = std::max(coverage.max_ratio, clamped_ratio);
    return true;
}

void update_path_coverage(const sim::Mission &mission,
                          const sim::Segment &segment,
                          sim::Point from,
                          sim::Point to,
                          Coverage &coverage) {
    const int samples = std::max(1, static_cast<int>(std::ceil(sim::distance(from, to) / 0.05)));
    for (int index = 0; index <= samples; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(samples);
        update_coverage(mission,
                        segment,
                        sim::Point{
                            .x = from.x + (to.x - from.x) * ratio,
                            .y = from.y + (to.y - from.y) * ratio,
                        },
                        coverage);
    }
}

}  // namespace

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    std::optional<sim::Mission> mission;
    std::map<std::string, Coverage> coverage;
    std::set<std::string> active_drive_over;
    sim::PaintReport report;
    LastPaintSample last_paint;

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
            if (mission) {
                report.all_complete = false;
                for (const auto &segment : mission->segments) {
                    coverage.try_emplace(segment.id);
                }
            }
            continue;
        }

        if (event->id == "paint_event" && mission) {
            const auto paint_event = sim::parse_paint_event(event->data);
            if (!paint_event) {
                continue;
            }
            const sim::Point nozzle{.x = paint_event->x, .y = paint_event->y};
            const sim::Point robot{.x = paint_event->robot_x, .y = paint_event->robot_y};

            for (const auto &segment : mission->segments) {
                if (paint_event->enabled && paint_event->line_id == segment.id) {
                    auto &entry = coverage[segment.id];
                    update_coverage(*mission, segment, nozzle, entry);
                    if (last_paint.valid && last_paint.line_id == segment.id) {
                        update_path_coverage(*mission, segment, last_paint.nozzle, nozzle, entry);
                    }
                    if (entry.max_ratio - entry.min_ratio >= 0.92) {
                        report.completed.insert(segment.id);
                    }
                }

                const double robot_lateral = std::abs(sim::signed_lateral_error(segment.start, segment.end, robot));
                const double robot_ratio = sim::projection_ratio(segment.start, segment.end, robot);
                const bool on_completed_interior = robot_lateral <= mission->completion_tolerance_m &&
                                                   robot_ratio > 0.05 && robot_ratio < 0.95 &&
                                                   report.completed.contains(segment.id);
                if (!paint_event->enabled && on_completed_interior) {
                    if (!active_drive_over.contains(segment.id)) {
                        report.drive_over_violations += 1;
                        active_drive_over.insert(segment.id);
                    }
                } else {
                    active_drive_over.erase(segment.id);
                }
            }

            if (paint_event->enabled) {
                last_paint = LastPaintSample{
                    .valid = true,
                    .line_id = paint_event->line_id,
                    .nozzle = nozzle,
                };
            } else {
                last_paint.valid = false;
            }
        }

        if (event->id == "tick" && mission) {
            report.all_complete = report.completed.size() == mission->segments.size();
            node.send("paint_report", sim::render_paint_report(report));
        }
    }
    return 0;
}
