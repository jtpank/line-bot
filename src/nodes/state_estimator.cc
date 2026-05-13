#include <optional>

#include "common/dora_node.hh"
#include "common/messages.hh"

int main() {
    sim::DoraNode node;
    if (!node.ok()) {
        return 1;
    }

    std::optional<sim::GpsFix> latest_gps;
    std::optional<sim::ImuSample> latest_imu;
    sim::RobotState estimate;

    while (true) {
        auto event = node.next();
        if (!event || event->kind == sim::EventKind::Stop) {
            break;
        }
        if (event->kind != sim::EventKind::Input) {
            continue;
        }

        if (event->id == "rtk_fix") {
            latest_gps = sim::parse_gps_fix(event->data);
        } else if (event->id == "imu") {
            latest_imu = sim::parse_imu(event->data);
        } else if (event->id == "tick") {
            if (latest_gps) {
                estimate.x = latest_gps->x;
                estimate.y = latest_gps->y;
                estimate.yaw = latest_gps->yaw;
            } else if (latest_imu) {
                estimate.yaw = latest_imu->yaw;
            }
            node.send("state_estimate", sim::render_state(estimate));
        }
    }
    return 0;
}
