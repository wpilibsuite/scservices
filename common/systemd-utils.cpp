#include "systemd-utils.h"

#ifdef MRC_DAEMON_BUILD
#include <systemd/sd-daemon.h>
#endif

void systemd_utils::notify_ready() {
#ifdef MRC_DAEMON_BUILD
    sd_notify(0, "READY=1");
#endif
}

void systemd_utils::notify_stopping() {
#ifdef MRC_DAEMON_BUILD
    sd_notify(0, "STOPPING=1");
#endif
}

void systemd_utils::notify_status(std::string_view status) {
#ifdef MRC_DAEMON_BUILD
    sd_notifyf(0, "STATUS=%s", status.data());
#endif
}
