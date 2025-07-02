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
#include <wpinet/uv/Timer.h>

#include "networktables/NetworkTableInstance.h"
#include "networktables/IntegerTopic.h"

#define NUM_CAN_BUSES 5

// Robot Controller, NI, 1023 API Id
static constexpr uint32_t crossMessageId = 0x101FFF8;
static constexpr uint32_t crossMessageMask = 0x1FFFFFF8;

struct CanState {
    int socketHandle{-1};
    nt::IntegerPublisher crossDetectPublisher;
    unsigned busId{0};
    uint32_t crossedBusses = 0;

    ~CanState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void handleCanFrame(const can_frame& frame);
    bool startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                     wpi::uv::Loop& loop);
};

void CanState::handleCanFrame(const can_frame& frame) {
    uint32_t deviceId = (frame.can_id & ~crossMessageId) & 0x7;

    crossedBusses |= (1 << deviceId);

    crossDetectPublisher.Set(crossedBusses);
}

bool CanState::startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                           wpi::uv::Loop& loop) {
    if (bus >= NUM_CAN_BUSES) {
        return false;
    }

    busId = bus;

    auto busIdStr = std::to_string(busId);

    crossDetectPublisher =
        ntInst.GetIntegerTopic("/cancross/" + busIdStr).Publish();
    crossDetectPublisher.Set(crossedBusses);

    socketHandle =
        socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

    if (socketHandle == -1) {
        return false;
    }

    struct can_filter filter{
        .can_id = crossMessageId | CAN_EFF_FLAG,
        .can_mask = crossMessageMask | CAN_EFF_FLAG,
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

    poll->pollEvent.connect([this](int mask) {
        if (mask & UV_READABLE) {
            can_frame frame;
            int rVal = read(socketHandle, &frame, sizeof(frame));

            if (rVal != CAN_MTU) {
                // TODO Error handling, do we need to reopen the socket?
                return;
            }

            if (frame.can_id & CAN_ERR_FLAG) {
                // Do nothing if this is an error frame
                return;
            }

            handleCanFrame(frame);
        }
    });

    auto timer = wpi::uv::Timer::Create(loop);
    if (!timer) {
        return false;
    }

    timer->timeout.connect([this]() {
        // If we didn't have a cross last iteration, write the value.
        if (crossedBusses == 0) {
            crossDetectPublisher.Set(crossedBusses);
        }
        crossedBusses = 0;

        can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = crossMessageId | busId | CAN_EFF_FLAG;
        write(socketHandle, &frame, sizeof(frame));
        // TODO do we need to error here?
    });

    poll->Start(UV_READABLE);

    // Write every 5 seconds.
    timer->Start(wpi::uv::Timer::Time{100}, wpi::uv::Timer::Time{5000});

    return true;
}

int main() {
    printf("Starting CanCrossStreamDetectorDaemon\n");
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
    ntInst.StartClient("CanCrossStreamDetectorDaemon");

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
