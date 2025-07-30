#include "EnabledState.h"

#include <fcntl.h>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <string.h>

#define CONTROL_DATA_PATH "/sys/kernel/can_heartbeat/controldataro"

namespace eh {

EnabledState::~EnabledState() {
    if (fd == -1) {
        return;
    }
    close(fd);
}

bool EnabledState::Initialize() {
    int retries = 0;
    while (fd == -1 && retries < 50) {
        fd = open(CONTROL_DATA_PATH, O_RDONLY);
        if (fd == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            retries++;
        }
    }

    return fd != -1;
}

bool EnabledState::IsEnabled() const {
    if (fd == -1) {
        return false;
    }

    char buf[128];

    ssize_t control_data_size = pread(fd, buf, sizeof(buf), 0);

    buf[control_data_size] = '\0';

    unsigned int control_data = strtol(buf, NULL, 16);

    bool system_watchdog = (control_data & 0x1) != 0 ? true : false;

    return system_watchdog;
}
}  // namespace eh
