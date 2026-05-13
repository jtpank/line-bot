#include <optional>

#include "common/dora_node.hh"
#include "common/geometry.hh"
#include "common/messages.hh"

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    std::optional<sim::RobotState> last_state;

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind != sim::EventKind::Input || event->id != "true_state") {
            continue;
        }
        auto state = sim::parse_state(event->data);
        if (!state) {
            continue;
        }

        sim::ImuSample imu{.yaw = state->yaw};
        if (last_state && state->t > last_state->t) {
            const double dt = state->t - last_state->t;
            imu.yaw_rate_rps = sim::wrap_angle(state->yaw - last_state->yaw) / dt;
            imu.accel_mps2 = (state->speed_mps - last_state->speed_mps) / dt;
        }
        last_state = *state;
        node.send("imu", sim::render_imu(imu));
    }
    return 0;
}
