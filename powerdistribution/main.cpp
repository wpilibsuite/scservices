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

#define NUM_CAN_BUSES 2

static constexpr uint32_t deviceTypeMask = 0x3F000000;
static constexpr uint32_t powerDistributionFilter = 0x08000000;

struct CanState {
    int socketHandle{-1};
    nt::IntegerPublisher deviceIdPublisher;
    std::array<nt::RawPublisher, 4> framePublishers;
    unsigned busId{0};

    ~CanState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void handleCanFrame(const canfd_frame& frame);
    void handlePowerFrame(const canfd_frame& frame);
    bool startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                     wpi::uv::Loop& loop);
};

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
    }
}

void CanState::handlePowerFrame(const canfd_frame& frame) {
    uint16_t apiId = (frame.can_id >> 6) & 0x3FF;

    int frameNum = 0;
    uint32_t deviceId = frame.can_id & 0x1FFF003F;

    if (frame.can_id & 0x10000) {
        // Rev Frame
        if (apiId < 0x60 || apiId > 0x63) {
            // Not valid
            return;
        }

        frameNum = apiId - 0x60;
    } else {
        // CTRE frame
        if (apiId == 0x5D) {
            // Special case
            frameNum = 3;
        } else if (apiId < 0x50 || apiId > 0x52) {
            // Not valid
            return;
        } else {
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
    if (bus >= NUM_CAN_BUSES) {
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

    socketHandle =
        socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

    if (socketHandle == -1) {
        return false;
    }

    // Filter to PD device type.
    // Both mfg types have the "4" bit set. They just
    // differ on the 1 bit. So a single filter can be used,
    // ignoring that bit.
    struct can_filter filter {
        .can_id = 0x08040000 | CAN_EFF_FLAG,
        .can_mask = 0x1FFE0000 | CAN_EFF_FLAG,
    };

    if (setsockopt(socketHandle, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                   sizeof(filter)) == -1) {
        return false;
    }

    ifreq ifr;
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can_s%u", busId);

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
    ntInst.StartClient("PowerDistributionDaemon");

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
