#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include "version.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <wpinet/EventLoopRunner.h>
#include <wpinet/uv/Poll.h>

#include "networktables/NetworkTableInstance.h"
#include "networktables/RawTopic.h"
#include "networktables/IntegerTopic.h"
#include "networktables/StringArrayTopic.h"

#define NUM_CAN_BUSES 5

static constexpr uint32_t deviceTypeMask = 0x3F000000;
static constexpr uint32_t powerDistributionFilter = 0x08000000;
static constexpr uint32_t pneumaticsFilter = 0x09000000;
static constexpr uint32_t manufacturerMask = 0x00FF0000;
static constexpr uint32_t ctreFilter = 0x00040000;
static constexpr uint32_t revFilter = 0x00050000;

struct SendVersionStore {
    uint8_t ctrepcm : 1;
    uint8_t ctrepdh : 1;
    uint8_t revpdh : 1;
    uint8_t revph : 1;
    uint8_t reserved : 4;
};

struct CanState {
    int socketHandle{-1};
    nt::IntegerPublisher deviceIdPublisher;
    std::array<nt::RawPublisher, 4> framePublishers;
    nt::StringArrayPublisher versionPublisher;
    std::array<SendVersionStore, 64> sentVersions{0, 0, 0, 0, 0};
    unsigned busId{0};

    ~CanState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void maybeSendRevVersionRequest(uint8_t deviceId, bool isPower);

    void handleRevVersionFrame(const canfd_frame& frame, bool isPower);

    void handleCanFrame(const canfd_frame& frame);
    void handlePowerFrame(const canfd_frame& frame);
    void handlePneumaticsFrame(const canfd_frame& frame);
    bool startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                     wpi::uv::Loop& loop);
};

void CanState::maybeSendRevVersionRequest(uint8_t deviceId, bool isPower) {
    if (deviceId > 63) {
        return;
    }

    if (isPower && sentVersions[deviceId].revpdh) {
        return;
    } else if (!isPower && sentVersions[deviceId].revph) {
        return;
    }

    constexpr uint32_t pneumaticsRequest = 0x09052600;
    constexpr uint32_t powerRequest = 0x08052600;

    canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = deviceId | CAN_EFF_FLAG | CAN_RTR_FLAG;
    frame.can_id |= isPower ? powerRequest : pneumaticsRequest;
    frame.len = 8;

    send(socketHandle, &frame, CAN_MTU, 0);
}

void CanState::handleRevVersionFrame(const canfd_frame& frame, bool isPower) {
    if (frame.len < 8) {
        return;
    }

    uint8_t year = frame.data[0];
    uint8_t minor = frame.data[1];
    uint8_t fix = frame.data[2];

    uint8_t deviceId = (frame.can_id & 0x3F);

    uint16_t deviceBusCombined = busId << 8 | deviceId;

    char buf[32];
    snprintf(buf, sizeof(buf), "%u.%u.%u", year, minor, fix);

    std::array<std::string, 3> sendData;
    sendData[0] = std::to_string(deviceBusCombined);
    sendData[1] = isPower ? "RevPH" : "RevPDH";
    sendData[2] = buf;

    versionPublisher.Set(sendData);

    if (isPower) {
        sentVersions[deviceId].revpdh = 1;
    } else {
        sentVersions[deviceId].revph = 1;
    }
}

void CanState::handlePneumaticsFrame(const canfd_frame& frame) {
    // The only thing we're doing with pneumatics is version receiving.
    constexpr uint32_t revPhVersionPacketMask = 0x09052600;
    constexpr uint32_t revPhAnyPacketMask = 0x09050000;

    if ((frame.can_id & revPhVersionPacketMask) == revPhVersionPacketMask) {
        handleRevVersionFrame(frame, false);
    } else if ((frame.can_id & revPhAnyPacketMask) == revPhAnyPacketMask) {
        maybeSendRevVersionRequest(frame.can_id & 0x3F, false);
    }
}

void CanState::handleCanFrame(const canfd_frame& frame) {
    // Can't support FD frames
    if (frame.flags & CANFD_FDF) {
        return;
    }

    // Looking for Device Type 8 or 9.
    // That will tell us what we're handling
    uint32_t maskedDeviceType = frame.can_id & deviceTypeMask;

    if (maskedDeviceType == powerDistributionFilter) {
        handlePowerFrame(frame);
    } else if (maskedDeviceType == pneumaticsFilter) {
        handlePneumaticsFrame(frame);
    }
}

void CanState::handlePowerFrame(const canfd_frame& frame) {
    uint16_t apiId = (frame.can_id >> 6) & 0x3FF;

    int frameNum = 0;
    uint32_t deviceId = frame.can_id & 0x1FFF003F;

    if (frame.can_id & 0x10000) {
        // Rev Frame
        if (apiId == 0x98) {
            // Version frame
            handleRevVersionFrame(frame, true);
            return;
        } else if (apiId < 0x60 || apiId > 0x63) {
            // Not valid
            return;
        }

        maybeSendRevVersionRequest(frame.can_id & 0x3F, true);

        frameNum = apiId - 0x60;
    } else {
        // CTRE frame

        if (apiId == 0x5D) {
            // Special case
            frameNum = 3;
        } else {
            if (apiId < 0x50 || apiId > 0x52) {
                // Not valid
                return;
            }

            frameNum = apiId - 0x50;
        }
    }

    deviceIdPublisher.Set(deviceId);

    std::span<const uint8_t> frameSpan = {
        reinterpret_cast<const uint8_t*>(frame.data), frame.len};

    if (frameNum < 0 || frameNum >= static_cast<int>(framePublishers.size())) {
        printf("BUGBUG logic error invalid frame number\n");
        return;
    }

    framePublishers[frameNum].Set(frameSpan);
}

bool CanState::startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                           wpi::uv::Loop& loop) {
    if (busId >= NUM_CAN_BUSES) {
        return false;
    }

    busId = bus;

    nt::PubSubOptions options;
    options.sendAll = true;
    options.keepDuplicates = true;
    options.periodic = 0.005;

    auto busIdStr = std::to_string(busId);

    for (size_t i = 0; i < framePublishers.size(); i++) {
        auto iStr = std::to_string(i);
        framePublishers[i] =
            ntInst.GetRawTopic("/pd/" + busIdStr + "/frame" + iStr)
                .Publish("pd", options);
    }

    deviceIdPublisher =
        ntInst.GetIntegerTopic("/pd/" + busIdStr + "/deviceid").Publish();

    versionPublisher =
        ntInst.GetStringArrayTopic("/Netcomm/Reporting/LibVersionStr")
            .Publish(options);

    socketHandle =
        socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

    if (socketHandle == -1) {
        return false;
    }

    // Filter to PD or pneumatics device type.
    // Both mfg types have the "4" bit set. They just
    // differ on the 1 bit. So a single filter can be used,
    // ignoring that bit.
    // Same thing for 8 vs 9 for the device type
    struct can_filter filter {
        .can_id = 0x08040000 | CAN_EFF_FLAG,
        .can_mask = 0x1EFE0000 | CAN_EFF_FLAG,
    };

    if (setsockopt(socketHandle, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                   sizeof(filter)) == -1) {
        return false;
    }

    ifreq ifr;
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can%u", busId);

    if (ioctl(socketHandle, SIOCGIFINDEX, &ifr) == -1) {
        return false;
    }

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketHandle, reinterpret_cast<const sockaddr*>(&addr),
             sizeof(addr)) == -1) {
        return false;
    }

    auto poll = wpi::uv::Poll::Create(loop, socketHandle);
    if (!poll) {
        return false;
    }

    poll->pollEvent.connect([this, fd = socketHandle](int mask) {
        if (mask & UV_READABLE) {
            canfd_frame frame;
            int rVal = read(fd, &frame, sizeof(frame));

            if (rVal != CAN_MTU && rVal != CANFD_MTU) {
                // TODO Error handling, do we need to reopen the socket?
                return;
            }

            if (frame.can_id & CAN_ERR_FLAG) {
                // Do nothing if this is an error frame
                return;
            }

            if (rVal == CANFD_MTU) {
                frame.flags = CANFD_FDF;
            }

            handleCanFrame(frame);
        }
    });

    poll->Start(UV_READABLE);

    return true;
}

int main() {
    printf("Starting PowerDistributionDaemon\n");
    printf("\tBuild Hash: %s\n", MRC_GetGitHash());
    printf("\tBuild Timestamp: %s\n", MRC_GetBuildTimestamp());

#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
#endif

    std::array<CanState, NUM_CAN_BUSES> states;

    auto ntInst = nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient4("PowerDistributionDaemon");

    wpi::EventLoopRunner loopRunner;

    bool success = false;
    loopRunner.ExecSync([&success, &states, &ntInst](wpi::uv::Loop& loop) {
        for (size_t i = 0; i < states.size(); i++) {
            success = states[i].startUvLoop(i, ntInst, loop);
            if (!success) {
                return;
            }
        }
    });

    if (!success) {
        loopRunner.Stop();
        return -1;
    }

    {
#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
        int sig = 0;
        sigwait(&signal_set, &sig);
#else
        (void)getchar();
#endif
    }
    ntInst.StopClient();
    nt::NetworkTableInstance::Destroy(ntInst);

    return 0;
}
