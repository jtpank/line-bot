#include "common/dora_node.hh"

#include <iostream>

namespace sim {
namespace {

EventKind to_event_kind(DoraEventType type) {
    switch (type) {
        case DoraEventType_Stop:
            return EventKind::Stop;
        case DoraEventType_Input:
            return EventKind::Input;
        case DoraEventType_InputClosed:
            return EventKind::InputClosed;
        case DoraEventType_Error:
            return EventKind::Error;
        case DoraEventType_Unknown:
        default:
            return EventKind::Unknown;
    }
}

std::string read_string(char *ptr, size_t len) {
    if (ptr == nullptr || len == 0) {
        return {};
    }
    return std::string(ptr, len);
}

}  // namespace

DoraNode::DoraNode() : context_(init_dora_context_from_env()) {
    if (context_ == nullptr) {
        std::cerr << "failed to initialize Dora context from environment\n";
    }
}

DoraNode::~DoraNode() {
    if (context_ != nullptr) {
        free_dora_context(context_);
    }
}

bool DoraNode::ok() const {
    return context_ != nullptr;
}

std::optional<DoraEvent> DoraNode::next() {
    if (context_ == nullptr) {
        return std::nullopt;
    }

    void *raw_event = dora_next_event(context_);
    if (raw_event == nullptr) {
        return std::nullopt;
    }

    DoraEvent event;
    event.kind = to_event_kind(read_dora_event_type(raw_event));
    event.timestamp = read_dora_input_timestamp(raw_event);

    if (event.kind == EventKind::Input) {
        char *id_ptr = nullptr;
        size_t id_len = 0;
        read_dora_input_id(raw_event, &id_ptr, &id_len);
        event.id = read_string(id_ptr, id_len);

        char *data_ptr = nullptr;
        size_t data_len = 0;
        read_dora_input_data(raw_event, &data_ptr, &data_len);
        event.data = read_string(data_ptr, data_len);
    }

    free_dora_event(raw_event);
    return event;
}

bool DoraNode::send(const std::string &id, const std::string &data) {
    if (context_ == nullptr) {
        return false;
    }

    auto *id_ptr = const_cast<char *>(id.data());
    auto *data_ptr = const_cast<char *>(data.data());
    const int result = dora_send_output(context_, id_ptr, id.size(), data_ptr, data.size());
    if (result != 0) {
        std::cerr << "failed to send Dora output '" << id << "'\n";
        return false;
    }
    return true;
}

}  // namespace sim
