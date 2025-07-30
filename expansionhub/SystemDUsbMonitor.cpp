#include "SystemDUsbMonitor.h"

#include "stdio.h"

#include <filesystem>

using namespace eh;

bool SystemDUsbMonitor::Initialize(wpi::uv::Loop* loop) {
    uvLoop = loop;
    sd_device_monitor* devMonitor = nullptr;
    int res = sd_device_monitor_new(&devMonitor);

    if (res < 0) {
        printf("sd_device_monitor_new failed %d\n", res);
        return false;
    }

    res = sd_device_monitor_filter_add_match_subsystem_devtype(
        devMonitor, "usb-serial", NULL);
    if (res < 0) {
        printf(
            "sd_device_monitor_filter_add_match_subsystem_devtype failed "
            "%d\n",
            res);
        return false;
    }

    res = sd_device_monitor_start(devMonitor, &RawMonitorHandler, this);

    if (res < 0) {
        printf("sd_device_monitor_start failed %d\n", res);
        return false;
    }

    sdEvent = sd_device_monitor_get_event(devMonitor);
    if (!sdEvent) {
        printf("sd_device_monitor_get_event failed\n");
        return false;
    }

    fd = sd_event_get_fd(sdEvent);
    if (fd < 0) {
        printf("sd_event_get_fd failed %d\n", fd);
        return false;
    }
    return true;
}

void SystemDUsbMonitor::HandleEvent() {
    if (sdEvent) {
        sd_event_run(sdEvent, 0);
    }
}

void SystemDUsbMonitor::DoInitialCheck() {
    sd_device_enumerator* devEnum;
    sd_device_enumerator_new(&devEnum);
    sd_device_enumerator_add_match_subsystem(devEnum, "usb-serial", 1);

    sd_device* newDev = sd_device_enumerator_get_device_first(devEnum);
    while (newDev) {
        const char* path = "UNKNOWN";
        sd_device_get_subsystem(newDev, &path);
        if (strcmp(path, "usb-serial") == 0) {
            OnDeviceAdded(newDev);
        }
        newDev = sd_device_enumerator_get_device_next(devEnum);
    }
}

int SystemDUsbMonitor::RawMonitorHandler(sd_device_monitor* m,
                                         sd_device* device, void* userdata) {
    return reinterpret_cast<SystemDUsbMonitor*>(userdata)->DeviceHandler(
        device);
}

int SystemDUsbMonitor::DeviceHandler(sd_device* device) {
    sd_device_action_t action = _SD_DEVICE_ACTION_INVALID;
    sd_device_get_action(device, &action);

    if (action == SD_DEVICE_BIND) {
        printf("Detected new device\n");
        OnDeviceAdded(device);
    } else if (action == SD_DEVICE_REMOVE) {
        OnDeviceRemoved(device);
    }

    return 0;
}

void SystemDUsbMonitor::OnDeviceAdded(sd_device* device) {
    int ret;

    const char* model = "UNKNOWN";

    sd_device* bridge = NULL;
    ret = sd_device_get_parent(device, &bridge);

    if (ret < 0 || bridge == NULL) {
        printf("No bridge\n");
        return;
    }

    sd_device* ftdi = NULL;
    ret = sd_device_get_parent(bridge, &ftdi);

    if (ret < 0 || ftdi == NULL) {
        printf("No ftdi\n");
        return;
    }

    ret = sd_device_get_property_value(ftdi, "ID_MODEL", &model);

    if (ret < 0) {
        printf("No ID_MODEL found\n");
        return;
    }

    if (strcmp(model, "FT230X_First_2.0") != 0) {
        printf("Unknown device %s\n", model);
    }

    const char* syspath = "unknown";
    sd_device_get_syspath(ftdi, &syspath);

    std::filesystem::path fullUsbPath{syspath};

    std::filesystem::path busPath = fullUsbPath.filename();

    std::string_view busPathView = busPath.native();

    if (!busPathView.starts_with("3-1.")) {
        printf("Invalid bus path, \"%s\" must start with \"3-1.\"\n",
               busPath.c_str());
        return;
    }

    int busNum = *(busPathView.end() - 1) - '1';

    if (busNum < 0 || busNum >= NUM_USB_BUSES) {
        printf("Invalid bus number %s\n", busPath.c_str());
        return;
    }

    const char* serialPath;
    ret = sd_device_get_syspath(device, &serialPath);
    if (ret < 0) {
        printf("No sd_device_get_syspath failed, no sys path\n");
        return;
    }

    std::filesystem::path fullTtyPath{serialPath};

    std::filesystem::path ttyPath = fullTtyPath.filename();

    std::filesystem::path devPath = "/dev";
    devPath.append(ttyPath.native());

    printf("Found device %s on bus num %d\n", devPath.c_str(), busNum);

    OnAdded(*uvLoop, busNum, devPath);
}

void SystemDUsbMonitor::OnDeviceRemoved(sd_device* device) {
    const char* serialPath;
    int ret = sd_device_get_syspath(device, &serialPath);
    if (ret < 0) {
        printf("No sd_device_get_syspath failed, no sys path\n");
        return;
    }

    std::filesystem::path fullTtyPath{serialPath};

    std::filesystem::path ttyPath = fullTtyPath.filename();

    std::filesystem::path devPath = "/dev";
    devPath.append(ttyPath.native());

    OnRemoved(devPath);
}
