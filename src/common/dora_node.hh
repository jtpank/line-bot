#pragma once

#include <optional>
#include <string>

#include "dora_node_api.h"

namespace sim {

enum class EventKind {
    Stop,
    Input,
    InputClosed,
    Error,
    Unknown,
};

struct DoraEvent {
    EventKind kind{EventKind::Unknown};
    std::string id;
    std::string data;
    unsigned long long timestamp{0};
};

class DoraNode {
  public:
    DoraNode();
    ~DoraNode();

    DoraNode(const DoraNode &) = delete;
    DoraNode &operator=(const DoraNode &) = delete;

    bool ok() const;
    std::optional<DoraEvent> next();
    bool send(const std::string &id, const std::string &data);

  private:
    void *context_{nullptr};
};

}  // namespace sim
