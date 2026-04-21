#pragma once

#include <string_view>
namespace systemd_utils {
void notify_ready();
void notify_stopping();
void notify_status(std::string_view status);
}  // namespace systemd_utils
