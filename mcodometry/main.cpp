#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include "version.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <wpi/util/mutex.hpp>

#include <wpi/util/print.hpp>

#include <wpi/net/EventLoopRunner.hpp>
#include <wpi/net/uv/Poll.hpp>

#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/nt/RawTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"

#include "wpi/util/Endian.hpp"

#define NUM_CAN_BUSES 1

static constexpr uint32_t motionCorePacketMask =
    0x10208FF;  // 00001 00000010 0000100011 111111
static constexpr uint32_t motionCorePacketFilter =
    0x10208C0;  // 00001 00000010 0000100011 000000

static constexpr uint32_t writeFrame = 0x010209C0;

struct MotioncoreMessage {
    uint64_t timestampUs;
    int32_t encoderPositions[3];
    int16_t encoderVelocities[3];
    uint8_t pinStates;
    uint8_t encoderWindows[3];
};

static_assert(sizeof(MotioncoreMessage) == 32,
              "MotioncoreMessage must be 32 bytes");
static_assert(offsetof(MotioncoreMessage, encoderWindows[2]) == 29,
              "encoderWindows[2] must be at offset 29");
static_assert(sizeof(MotioncoreMessage::encoderWindows[2]) == 1,
              "encoderWindows[2] must be 1 byte");

struct CanState {
    int socketHandle{-1};
    unsigned busId{0};

    wpi::nt::RawPublisher messagePublisher;
    std::array<wpi::nt::IntegerSubscriber, 3> encoderPositionSubscribers;
    std::array<std::optional<uint64_t>, 3> lastEncoderPositionSetTimes;

    std::array<wpi::nt::IntegerSubscriber, 3> encoderVelocityWindowSubscribers;
    std::array<std::optional<uint8_t>, 3> encoderWindows;

    ~CanState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void handleCanFrame(const canfd_frame& frame);
    void sendWindowFrame(size_t index, uint8_t window);
    void sendEncoderPositionFrame(size_t index, int32_t position);
    bool startUvLoop(unsigned bus, const wpi::nt::NetworkTableInstance& ntInst,
                     wpi::net::uv::Loop& loop);

    MotioncoreMessage latestMessage;
};

void CanState::handleCanFrame(const canfd_frame& frame) {
    if ((frame.can_id & motionCorePacketMask) != motionCorePacketFilter) {
        // Not a motion core packet, ignore
        return;
    }

    if (frame.len != 32) {
        // Not a valid motion core packet, ignore
        return;
    }

    const unsigned char* dataPtr = frame.data;
    latestMessage.timestampUs = wpi::util::support::endian::readNext<uint64_t>(
        dataPtr, wpi::util::endianness::big);
    latestMessage.encoderPositions[0] =
        wpi::util::support::endian::readNext<int32_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderPositions[1] =
        wpi::util::support::endian::readNext<int32_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderPositions[2] =
        wpi::util::support::endian::readNext<int32_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderVelocities[0] =
        wpi::util::support::endian::readNext<int16_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderVelocities[1] =
        wpi::util::support::endian::readNext<int16_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderVelocities[2] =
        wpi::util::support::endian::readNext<int16_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.pinStates = wpi::util::support::endian::readNext<uint8_t>(
        dataPtr, wpi::util::endianness::big);
    latestMessage.encoderWindows[0] =
        wpi::util::support::endian::readNext<uint8_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderWindows[1] =
        wpi::util::support::endian::readNext<uint8_t>(
            dataPtr, wpi::util::endianness::big);
    latestMessage.encoderWindows[2] =
        wpi::util::support::endian::readNext<uint8_t>(
            dataPtr, wpi::util::endianness::big);

    messagePublisher.Set(
        {reinterpret_cast<const uint8_t*>(&latestMessage), 30});

    for (size_t i = 0; i < encoderPositionSubscribers.size(); i++) {
        auto newPos = encoderPositionSubscribers[i].GetAtomic(INT64_MAX);
        if (newPos.time == 0) {
            // Definitely nothing to do, return
            continue;
        }
        if (!lastEncoderPositionSetTimes[i].has_value()) {
            // We're doing our first set here.
            lastEncoderPositionSetTimes[i] = newPos.time;
            sendEncoderPositionFrame(i, static_cast<int32_t>(newPos.value));
            continue;
        }
        if (newPos.time != lastEncoderPositionSetTimes[i]) {
            // This is a new value, send it out
            lastEncoderPositionSetTimes[i] = newPos.time;
            sendEncoderPositionFrame(i, static_cast<int32_t>(newPos.value));
        }
    }

    for (size_t i = 0; i < encoderWindows.size(); i++) {
        if (!encoderWindows[i].has_value()) {
            encoderWindows[i] = latestMessage.encoderWindows[i];
        }
        // Get the set encoder window if it exists.
        auto ntWindow = encoderVelocityWindowSubscribers[i].Get(-1);

        if (ntWindow >= 0 && ntWindow <= 255) {
            encoderWindows[i] = static_cast<uint8_t>(ntWindow);
        }

        if (latestMessage.encoderWindows[i] != encoderWindows[i]) {
            sendWindowFrame(i, encoderWindows[i].value());
        }
    }
}

void CanState::sendWindowFrame(size_t index, uint8_t window) {
    canfd_frame frame{};
    frame.can_id = writeFrame | CAN_EFF_FLAG;
    frame.len = 8;
    frame.flags =
        CANFD_FDF | CANFD_BRS;  // Use CAN FD frame format for larger payload

    frame.data[0] = static_cast<unsigned char>(index && 0xFF);
    frame.data[1] = 0x01;  // Subcommand for setting encoder window
    frame.data[2] = window;

    int rVal = write(socketHandle, &frame, CANFD_MTU);
    if (rVal != CANFD_MTU) {
        // TODO error handling, do we need to reopen the socket?
        printf("Error sending window frame\n");
    }
}

void CanState::sendEncoderPositionFrame(size_t index, int32_t position) {
    canfd_frame frame{};
    frame.can_id = writeFrame | CAN_EFF_FLAG;
    frame.len = 8;
    frame.flags =
        CANFD_FDF | CANFD_BRS;  // Use CAN FD frame format for larger payload

    frame.data[0] = static_cast<unsigned char>(index && 0xFF);
    frame.data[1] = 0x02;  // Subcommand for setting encoder position

    wpi::util::support::endian::write32be(frame.data + 4,
                                          static_cast<uint32_t>(position));

    int rVal = write(socketHandle, &frame, CANFD_MTU);
    if (rVal != CANFD_MTU) {
        // TODO error handling, do we need to reopen the socket?
        printf("Error sending encoder position frame\n");
    }
}

bool CanState::startUvLoop(unsigned bus,
                           const wpi::nt::NetworkTableInstance& ntInst,
                           wpi::net::uv::Loop& loop) {
    if (bus >= NUM_CAN_BUSES) {
        return false;
    }

    ifreq ifr;
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can_s%u", busId);

    wpi::nt::PubSubOptions options;
    options.periodic = 0.005;
    options.keepDuplicates = true;
    options.sendAll = true;

    messagePublisher = ntInst.GetRawTopic("/Motioncore/enc/data")
                           .Publish("motioncoreenc", options);

    for (size_t i = 0; i < encoderPositionSubscribers.size(); i++) {
        auto iStr = std::to_string(i);
        encoderPositionSubscribers[i] =
            ntInst.GetIntegerTopic("/Motioncore/enc/" + iStr + "/resetEncoder")
                .Subscribe(INT64_MAX, options);
        encoderVelocityWindowSubscribers[i] =
            ntInst
                .GetIntegerTopic("/Motioncore/enc/" + iStr + "/velocityWindow")
                .Subscribe(-1, options);
    }

    socketHandle =
        socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

    if (socketHandle == -1) {
        wpi::util::print("socket() for CAN {} failed with {}\n", ifr.ifr_name,
                         std::strerror(errno));
        return false;
    }

    struct can_filter filter{
        .can_id = motionCorePacketFilter | CAN_EFF_FLAG,
        .can_mask = motionCorePacketMask | CAN_EFF_FLAG,
    };

    if (setsockopt(socketHandle, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                   sizeof(filter)) == -1) {
        wpi::util::print(
            "setsockopt(CAN_RAW_FILTER) for CAN {} failed with {}\n",
            ifr.ifr_name, std::strerror(errno));
        return false;
    }

    if (ioctl(socketHandle, SIOCGIFINDEX, &ifr) == -1) {
        wpi::util::print("ioctl(SIOCGIFINDEX) for CAN {} failed with {}\n",
                         ifr.ifr_name, std::strerror(errno));
        return false;
    }

    int fdSet = 1;
    if (setsockopt(socketHandle, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fdSet,
                   sizeof(fdSet)) != 0) {
        wpi::util::print(
            "setsockopt(CAN_RAW_FD_FRAMES) for CAN {} failed with {}\n",
            ifr.ifr_name, std::strerror(errno));
        return false;
    }

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketHandle, reinterpret_cast<const sockaddr*>(&addr),
             sizeof(addr)) == -1) {
        wpi::util::print("bind() for CAN {} failed with {}\n", ifr.ifr_name,
                         std::strerror(errno));
        return false;
    }

    auto poll = wpi::net::uv::Poll::Create(loop, socketHandle);
    if (!poll) {
        wpi::util::print("Uv Poll for CAN {} failed\n", ifr.ifr_name);
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
    printf("Starting MotioncoreOdometryDaemon\n");
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

    auto ntInst = wpi::nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient("MotioncoreOdometryDaemon");

    wpi::net::EventLoopRunner loopRunner;
    bool success = false;
    loopRunner.ExecSync([&success, &states, &ntInst](wpi::net::uv::Loop& loop) {
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
    wpi::nt::NetworkTableInstance::Destroy(ntInst);

    return 0;
}
