#pragma once

#include "systemd/sd-device.h"
#include <optional>
#include <utility>
#include <functional>
#include <string>

#define NUM_USB_BUSES 4

namespace wpi::uv {
class Loop;
}

namespace eh {

class SystemDUsbMonitor {
   public:
    SystemDUsbMonitor(std::function<void(wpi::uv::Loop&, int, const std::string&)> added,
                      std::function<void(const std::string&)> removed)
        : OnAdded{std::move(added)}, OnRemoved{std::move(removed)} {}

    bool Initialize(wpi::uv::Loop* loop);

    int GetFd() { return fd; }

    void HandleEvent();

    void DoInitialCheck();

   private:
    std::function<void(wpi::uv::Loop&, int, const std::string&)> OnAdded;
    std::function<void(const std::string&)> OnRemoved;

    void OnDeviceAdded(sd_device* device);
    void OnDeviceRemoved(sd_device* device);

    static int RawMonitorHandler(sd_device_monitor* m, sd_device* device,
                                 void* userdata);
    int DeviceHandler(sd_device* device);

    int fd = -1;
    wpi::uv::Loop* uvLoop = nullptr;
    sd_event* sdEvent = nullptr;
};

}  // namespace eh
