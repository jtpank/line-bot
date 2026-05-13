#include "common/dora_node.hh"
#include "common/messages.hh"

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind == sim::EventKind::Input && event->id == "mission_done") {
            break;
        }
        if (event->kind == sim::EventKind::Input && event->id == "true_state") {
            if (auto state = sim::parse_state(event->data)) {
                node.send("rtk_fix", sim::render_gps_fix(sim::GpsFix{
                                      .x = state->x,
                                      .y = state->y,
                                      .yaw = state->yaw,
                                      .fix = "rtk",
                                  }));
            }
        }
    }
    return 0;
}
