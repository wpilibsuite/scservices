#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include "version.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <filesystem>

#include <wpinet/EventLoopRunner.h>
#include <wpinet/uv/Poll.h>
#include <wpinet/uv/FsEvent.h>
#include <wpinet/uv/Timer.h>

#include "networktables/NetworkTableInstance.h"
#include "networktables/RawTopic.h"

#include "networktables/IntegerTopic.h"
#include "networktables/DoubleTopic.h"

#include "systemd/sd-device.h"

#include "serial.h"

#include <deque>

#include "UvSerial.h"

#include "networktables/BooleanTopic.h"

#include <wpi/timestamp.h>

#include "ReceiveStateMachine.h"
#include "MessageNumbers.h"

struct robot_state {
    uint8_t tournament_type;  // 3 bits
    uint8_t system_watchdog;  // 1 bit
    uint8_t test_mode;        // 1 bit
    uint8_t autonomous;       // 1 bit
    uint8_t enabled;          // 1 bit
    uint8_t red_alliance;     // 1 bit
    uint8_t replay_number;    // 6 bits
    uint16_t match_number;    // 10 bits
    uint16_t
        match_time_seconds;  // 8 bits to CAN, 16 bits to other sysfs consumers
};

#define CONTROL_DATA_PATH "/sys/kernel/can_heartbeat/controldataro"

#define NUM_USB_BUSES 4
#define NUM_MOTORS_PER_HUB 4
#define NUM_SERVOS_PER_HUB 6
#define MAX_NUM_OUTSTANDING_MESSAGES 8

#define MODULE_STATUS_ID 0x7F03
#define KEEP_ALIVE_ID 0x7F04
#define QUERY_INTERFACE_ID 0x7F07
#define INTERFACE_STRING "DEKA"
#define DISCOVER_ID 0x7F0F
#define DISCOVER_ADDRESS 0xFF

#define MOTOR_0_ADC 8
#define BATTERY_ADC 13
#define POWER_CONVERSION 32767

#define MESSAGE_TIMEOUT 1000000

namespace eh {

struct MotorStore {
    nt::BooleanSubscriber enabledSubscriber;
    nt::DoublePublisher encoderPublisher;
    nt::DoublePublisher velocityPublisher;
    nt::DoublePublisher currentPublisher;

    nt::BooleanSubscriber floatOn0Subscriber;
    nt::IntegerSubscriber modeSubscriber;

    nt::DoubleSubscriber setpointSubscriber;

    nt::DoubleSubscriber pSubscriber;
    nt::DoubleSubscriber iSubscriber;
    nt::DoubleSubscriber dSubscriber;
    nt::DoubleSubscriber ffSubscriber;

    nt::BooleanSubscriber continousSubscriber;
    nt::BooleanSubscriber reversedSubscriber;
    nt::BooleanSubscriber resetEncoderSubscriber;

    void Initialize(const nt::NetworkTableInstance& instance, int motorNum,
                    const std::string& busIdStr, nt::PubSubOptions options);
};

void MotorStore::Initialize(const nt::NetworkTableInstance& instance,
                            int motorNum, const std::string& busIdStr,
                            nt::PubSubOptions options) {
    auto iStr = std::to_string(motorNum);
    encoderPublisher =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/encoder")
            .Publish(options);
    velocityPublisher = instance
                            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                            iStr + "/encoderVelocity")
                            .Publish(options);
    currentPublisher =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/current")
            .Publish(options);
    setpointSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/setpoint")
            .Subscribe(0, options);
    floatOn0Subscriber = instance
                             .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              iStr + "/floatOn0")
                             .Subscribe(false, options);
    enabledSubscriber =
        instance
            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/enabled")
            .Subscribe(false, options);

    modeSubscriber =
        instance
            .GetIntegerTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/mode")
            .Subscribe(0, options);

    pSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/pid/p")
            .Subscribe(0, options);

    iSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/pid/i")
            .Subscribe(0, options);

    dSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/pid/d")
            .Subscribe(0, options);

    ffSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr + "/pid/ff")
            .Subscribe(0, options);

    continousSubscriber = instance
                              .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                               iStr + "/pid/continous")
                              .Subscribe(false, options);

    reversedSubscriber = instance
                             .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              iStr + "/reversed")
                             .Subscribe(false, options);

    resetEncoderSubscriber = instance.GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              iStr + "/resetEncoder")
                             .Subscribe(false, options);
}

struct ServoStore {
    nt::BooleanSubscriber enabledSubscriber;
    nt::IntegerSubscriber pulseWidthSubscriber;
    nt::IntegerSubscriber framePeriodSubscriber;

    void Initialize(const nt::NetworkTableInstance& instance, int servoNum,
                    const std::string& busIdStr, nt::PubSubOptions options);
};

void ServoStore::Initialize(const nt::NetworkTableInstance& instance,
                            int servoNum, const std::string& busIdStr,
                            nt::PubSubOptions options) {
    auto iStr = std::to_string(servoNum);
    enabledSubscriber =
        instance
            .GetBooleanTopic("/rhsp/" + busIdStr + "/servo" + iStr + "/enabled")
            .Subscribe(false, options);
    framePeriodSubscriber =
        instance
            .GetIntegerTopic("/rhsp/" + busIdStr + "/servo" + iStr +
                             "/framePeriod")
            .Subscribe(0, options);
    pulseWidthSubscriber = instance
                               .GetIntegerTopic("/rhsp/" + busIdStr + "/servo" +
                                                iStr + "/pulseWidth")
                               .Subscribe(0, options);
}

struct NetworkTablesStore {
    std::array<MotorStore, NUM_MOTORS_PER_HUB> motors;
    std::array<ServoStore, NUM_SERVOS_PER_HUB> servos;

    nt::DoublePublisher batteryVoltagePublisher;

    nt::BooleanPublisher isConnectedPublisher;

    nt::IntegerPublisher numNacksPublisher;
    nt::IntegerPublisher numCrcFailuresPublisher;
    nt::IntegerPublisher numMissedSendLoopsPublisher;

    uint64_t numNacks{0};
    uint64_t numCrcFailures{0};
    uint64_t numMissedSendLoops{0};

    void Initialize(const nt::NetworkTableInstance& instance, int deviceNum);
};

void NetworkTablesStore::Initialize(const nt::NetworkTableInstance& instance,
                                    int deviceNum) {
    if (isConnectedPublisher) {
        return;
    }

    nt::PubSubOptions options;
    options.sendAll = true;
    options.keepDuplicates = true;
    options.periodic = 0.005;

    auto busIdStr = std::to_string(deviceNum);

    for (int i = 0; i < static_cast<int>(motors.size()); i++) {
        motors[i].Initialize(instance, i, busIdStr, options);
    }

    for (int i = 0; i < static_cast<int>(servos.size()); i++) {
        servos[i].Initialize(instance, i, busIdStr, options);
    }

    batteryVoltagePublisher =
        instance.GetDoubleTopic("/rhsp/" + busIdStr + "/battery")
            .Publish(options);
    isConnectedPublisher =
        instance.GetBooleanTopic("/rhsp/" + busIdStr + "/connected")
            .Publish(options);

    numCrcFailuresPublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/numCrcFailures")
            .Publish(options);

    numMissedSendLoopsPublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/numMissedSendLoops")
            .Publish(options);

    numNacksPublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/numNacks")
            .Publish(options);
}

static constexpr void WriteUint16(std::span<uint8_t> buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8);
}

static constexpr int16_t ReadInt16(std::span<const uint8_t> buffer) {
    return static_cast<int16_t>(buffer[0]) |
           (static_cast<int16_t>(buffer[1]) << 8);
}

static constexpr int32_t ReadInt32(std::span<const uint8_t> buffer) {
    return static_cast<int32_t>(buffer[0]) |
           (static_cast<int32_t>(buffer[1]) << 8) |
           (static_cast<int32_t>(buffer[2]) << 16) |
           (static_cast<int32_t>(buffer[3]) << 24);
}

static constexpr uint16_t ReadUint16(std::span<const uint8_t> buffer) {
    return static_cast<uint16_t>(buffer[0] |
                                 (static_cast<uint16_t>(buffer[1]) << 8));
}

static constexpr uint8_t PacketReferenceNumber(
    std::span<const uint8_t> buffer) {
    return buffer[7];
}

static constexpr uint8_t PacketDestinationAddress(
    std::span<const uint8_t> buffer) {
    return buffer[4];
}

static constexpr uint8_t PacketSourceAddress(std::span<const uint8_t> buffer) {
    return buffer[5];
}

static constexpr uint16_t PacketId(std::span<const uint8_t> buffer) {
    return ((uint16_t)(buffer[9]) << 8 | (uint16_t)(buffer[8]));
}

static constexpr std::span<const uint8_t> PacketPayloadBuffer(
    std::span<const uint8_t> buffer) {
    return buffer.subspan(10, buffer.size() - 11);
}

static constexpr uint8_t PacketCrc(std::span<const uint8_t> buffer) {
    return buffer[buffer.size() - 1];
}

static constexpr bool PacketIsAck(uint16_t packetId) {
    return packetId == 0x7F01;
}

static constexpr bool PacketIsNack(uint16_t packetId) {
    return packetId == 0x7F02;
}

static constexpr bool PacketIsDiscover(uint16_t packetId) {
    return packetId == (DISCOVER_ID | 0x8000);
}

static constexpr bool PacketIsQueryInterface(uint16_t packetId) {
    return packetId == (QUERY_INTERFACE_ID | 0x8000);
}

static constexpr uint8_t CalcChecksum(std::span<const uint8_t> buffer) {
    uint8_t sum = 0;
    for (auto&& b : buffer) {
        sum += b;
    }
    return sum;
}

enum class SendState {
    ReadyToSend,
    WaitingForEither,
    WaitingForBattery,
    WaitingForBulk,
    WaitingForFinish,
};

struct UvSerial {
    // Hardocde UART parameters
    UvSerial() = default;
    bool initialize(wpi::uv::Loop& loop, int fd) {
        serialFd = fd;
        tcflush(serialFd, TCIFLUSH);

        auto poll = wpi::uv::Poll::Create(loop, serialFd);
        if (!poll) {
            return false;
        }

        poll->pollEvent.connect([this](int flags) {
            if ((flags & UV_READABLE) != 0) {
                DoRead();
            }
        });

        poll->Start(UV_READABLE);

        return true;
    }

    void SetCallbacks(std::function<void(bool)> doOnSendCommands,
                      NetworkTablesStore* store) {
        onSendCommands = std::move(doOnSendCommands), ntStore = store;
    }

    NetworkTablesStore* ntStore{nullptr};
    std::function<void(bool)> onSendCommands;

    void RunDiscoverInternal() {
        auto now = wpi::Now();
        auto delta = now - discoverStartTime;

        // Don't try again
        if (delta <= MESSAGE_TIMEOUT) {
            return;
        }

        discoverStartTime = now;

        SendPacket(DISCOVER_ADDRESS, MESSAGE_DISCOVER, DISCOVER_ID, {}, true);
    }

    void RunInterfacePacketIdInternal() {
        auto now = wpi::Now();
        auto delta = now - discoverStartTime;

        // Don't try again
        if (delta <= MESSAGE_TIMEOUT) {
            return;
        }

        discoverStartTime = now;

        std::string_view interfaceString = INTERFACE_STRING;

        SendPacket(*address, MESSAGE_QUERY_INTERFACE, QUERY_INTERFACE_ID,
                   std::span<const uint8_t>{
                       reinterpret_cast<const uint8_t*>(interfaceString.data()),
                       (interfaceString.size() + 1)},
                   true);
    }

    void RunFtdiConfigureInternal() {
        auto now = wpi::Now();
        auto delta = now - discoverStartTime;

        // Don't try again
        if (delta <= MESSAGE_TIMEOUT) {
            return;
        }

        discoverStartTime = now;
        uint16_t packetId = *packetInterfaceId + 49;
        uint8_t buffer[1] = {1};

        SendPacket(*address, MESSAGE_FTDI_RESET_CONTROL, packetId, buffer,
                   true);
    }

    void RunSynchronousSteps() {
        if (!address.has_value()) {
            RunDiscoverInternal();
        } else if (!packetInterfaceId.has_value()) {
            RunInterfacePacketIdInternal();
        } else if (!configuredFtdiReset) {
            RunFtdiConfigureInternal();
        } else {
            printf("Device ready for transactions\n");
        }
    }

    bool HasFinishedSynchronous() { return configuredFtdiReset; }

    void SendBatteryRequest() {
        uint16_t packetId = *packetInterfaceId + 7;
        uint8_t buffer[2] = {BATTERY_ADC, 0};
        SendPacket(*address, MESSAGE_BATTERY_VOLTAGE, packetId, buffer);
    }

    void SendMotorCurrentRequest(uint8_t channel) {
        uint16_t packetId = *packetInterfaceId + 7;
        uint8_t adcChannel = MOTOR_0_ADC + channel;
        uint8_t buffer[2] = {adcChannel, 0};
        SendPacket(*address, MESSAGE_MOTOR_GET_CURRENT_0 + channel, packetId,
                   buffer);
    }

    void SendServoConfiguration(uint8_t channel, uint16_t framePeriod) {
        if (framePeriod <= 1) {
            return;
        }
        uint16_t packetId = *packetInterfaceId + 31;

        uint8_t buffer[3] = {channel, (uint8_t)(framePeriod),
                             (uint8_t)(framePeriod >> 8)};

        SendPacket(*address, MESSAGE_SERVO_CONFIGURATION_0 + channel, packetId,
                   buffer);
    }

    void SendServoPulseWidth(uint8_t channel, uint16_t pulseWidth) {
        if (pulseWidth == 0) {
            return;
        }
        uint16_t packetId = *packetInterfaceId + 33;

        uint8_t buffer[3] = {channel, (uint8_t)(pulseWidth),
                             (uint8_t)(pulseWidth >> 8)};

        SendPacket(*address, MESSAGE_SERVO_PULSE_WIDTH_0 + channel, packetId,
                   buffer);
    }

    void SendServoEnable(uint8_t channel, bool enable) {
        uint16_t packetId = *packetInterfaceId + 35;
        uint8_t enableVal = enable ? 1 : 0;
        uint8_t buffer[2] = {channel, enableVal};
        SendPacket(*address, MESSAGE_SERVO_ENABLE_0 + channel, packetId,
                   buffer);
    }

    void SendMotorConstantPower(uint8_t channel, double power) {
        power = std::clamp(power, -1.0, 1.0);

        uint16_t packetId = *packetInterfaceId + 15;

        int16_t adjustedPowerLevel = (int16_t)(power * POWER_CONVERSION);

        uint8_t buffer[3] = {channel, (uint8_t)(adjustedPowerLevel),
                             (uint8_t)(adjustedPowerLevel >> 8)};

        SendPacket(*address, MESSAGE_MOTOR_SET_CONSTANT_POWER_0 + channel,
                   packetId, buffer);
    }

    void SendMotorMode(uint8_t channel, bool floatVal) {
        uint16_t packetId = *packetInterfaceId + 8;
        uint8_t doFloat = floatVal ? 1 : 0;
        uint8_t cmdPayload[3] = {channel, 0, doFloat};
        SendPacket(*address, MESSAGE_MOTOR_SET_RUN_MODE_0 + channel, packetId,
                   cmdPayload);
    }

    void SendMotorEnable(uint8_t channel, bool enable) {
        uint16_t packetId = *packetInterfaceId + 10;
        uint8_t enableVal = enable ? 1 : 0;
        uint8_t buffer[2] = {channel, enableVal};
        SendPacket(*address, MESSAGE_MOTOR_ENABLE_0 + channel, packetId,
                   buffer);
    }

    void GetModuleStatus() {
        uint8_t clear = 1;
        SendPacket(*address, MESSAGE_MODULE_STATUS, MODULE_STATUS_ID,
                   std::span<const uint8_t>{&clear, 1});
    }

    void SendKeepAlive() {
        SendPacket(*address, MESSAGE_KEEP_ALIVE, KEEP_ALIVE_ID, {});
    }

    void SendBulkInput() {
        uint16_t packetId = *packetInterfaceId + 0;
        SendPacket(*address, MESSAGE_BULK_INPUT, packetId, {});
    }

    ~UvSerial() noexcept {
        auto lock = serialPoll.lock();
        if (lock) {
            printf("Closing serial poller\n");
            lock->Stop();
            lock->Close();
        }
        if (serialFd != -1) {
            printf("Closed fd\n");
            close(serialFd);
        }
    }

    void SendPacket(uint8_t destAddr, uint8_t messageNumber,
                    uint16_t packetTypeId, std::span<const uint8_t> payload,
                    bool direct = false) {
        assert(payload.size() < (1024 - 11));
        uint16_t bytesToSend = 10 + payload.size() + 1;

        std::span<uint8_t> txBufferSpan = txBuffer;
        txBufferSpan = txBufferSpan.subspan(0, bytesToSend);
        txBufferSpan[0] = 0x44;
        txBufferSpan[1] = 0x4B;

        WriteUint16(txBufferSpan.subspan(2, 2), bytesToSend);
        txBufferSpan[4] = destAddr;
        txBufferSpan[5] = 0x00;
        txBufferSpan[6] = messageNumber;
        txBufferSpan[7] = 0x00;
        WriteUint16(txBufferSpan.subspan(8, 2), packetTypeId);

        if (!payload.empty()) {
            memcpy(&txBufferSpan[10], payload.data(), payload.size());
        }
        txBufferSpan[10 + payload.size()] =
            CalcChecksum(txBufferSpan.subspan(0, 10 + payload.size()));

        if (direct) {
            write(serialFd, txBufferSpan.data(), txBufferSpan.size());
        } else {
            writeBuffer.insert(writeBuffer.end(), txBufferSpan.begin(),
                               txBufferSpan.end());
            pendingWrites.emplace_back(txBufferSpan.size());
        }
    }

    void DoRead() {
        ssize_t readVal = read(serialFd, readBuf, sizeof(readBuf));
        if (readVal <= 0) {
            // Error, do something.
            printf("Read error\n");
            return;
        }

        // Send bytes up
        stateMachine.HandleBytes(
            std::span<const uint8_t>{readBuf, static_cast<size_t>(readVal)});
        Flush();
    }

    bool AllowSend() { return sendState == SendState::ReadyToSend; }

    void StartTransaction(bool canDoEnable) {
        writeBuffer.clear();
        currentCount = 0;
        receivedCount = 0;
        lastLoop = wpi::Now();
        canEnable = canDoEnable;
        sendState = SendState::WaitingForEither;
    }

    void Flush() {
        size_t available = pendingWrites.size();
        size_t allowed = MAX_NUM_OUTSTANDING_MESSAGES - outstandingMessages;
        size_t toWrite = (std::min)(allowed, available);
        size_t count = 0;
        for (size_t i = 0; i < toWrite; i++) {
            count += pendingWrites.front();
            pendingWrites.pop_front();
        }
        if (count == 0) {
            return;
        }

        write(serialFd, writeBuffer.data() + currentCount, count);
        outstandingMessages += toWrite;
        currentCount += count;
    }

    void CheckForStateAdvance(MessageNumbers messageNumber, size_t dataSize) {
        receivedCount += dataSize;
        outstandingMessages--;

        if (messageNumber == MESSAGE_BATTERY_VOLTAGE &&
            sendState == SendState::WaitingForEither) {
            sendState = SendState::WaitingForBulk;
        } else if (messageNumber == MESSAGE_BULK_INPUT &&
                   sendState == SendState::WaitingForEither) {
            sendState = SendState::WaitingForBattery;
        } else if ((messageNumber == MESSAGE_BATTERY_VOLTAGE &&
                    sendState == SendState::WaitingForBattery) ||
                   (messageNumber == MESSAGE_BULK_INPUT &&
                    sendState == SendState::WaitingForBulk)) {
            // Have all the data needed to start commands.
            if (onSendCommands) {
                onSendCommands(canEnable);
            }
            sendState = SendState::WaitingForFinish;
        }
        if (pendingWrites.empty() && outstandingMessages == 0) {
            if (sendState != SendState::WaitingForFinish) {
                printf("Commands did not occur this loop\n");
            }
            // Done ready to send
            auto delta = wpi::Now() - lastLoop;
            if (delta >= 18000) {
                printf("Time to finish %ld %ld %ld\n", delta, currentCount,
                       receivedCount);
            }
            sendState = SendState::ReadyToSend;
        }
    }

    void HandlePayload(std::span<const uint8_t> data, uint8_t crc) {
        if (crc != PacketCrc(data)) {
            printf("CRC failure, bus will recover\n");
            if (ntStore) {
                ntStore->numCrcFailures++;
                ntStore->numCrcFailuresPublisher.Set(ntStore->numCrcFailures);
            }
            if (packetInterfaceId.has_value()) {
                // If we've already finished everything synchronous,
                // we can treat as a NACK.
                // The only case this isn't correct is if we get a crc
                // error on an RS485 connected hub.
                // This is both an unsupported scenario, and will just result
                // in undercounting (which will still recover, just longer)
                CheckForStateAdvance(MESSAGE_UNKNOWN, data.size());
            }
            return;
        }

        uint16_t packetId = PacketId(data);
        uint8_t packetReferenceNumber = PacketReferenceNumber(data);
        auto payload = PacketPayloadBuffer(data);

        if (PacketIsNack(packetId)) {
            if (packetReferenceNumber != MESSAGE_DISCOVER &&
                packetReferenceNumber != MESSAGE_QUERY_INTERFACE) {
                // Not a synchronous packet, adjust counts
                CheckForStateAdvance(MESSAGE_UNKNOWN, data.size());
            }
            printf("Nack %d for message id %d\n", payload[0],
                   packetReferenceNumber);
            if (ntStore) {
                ntStore->numNacks++;
                ntStore->numNacksPublisher.Set(ntStore->numNacks);
            }
            return;
        }

        if (PacketIsDiscover(packetId)) {
            if (!address.has_value() && payload[0] == 1) {
                address = PacketSourceAddress(data);
                discoverStartTime = 0;
                RunSynchronousSteps();
            }
            return;
        } else if (PacketIsAck(packetId) &&
                   packetReferenceNumber == MESSAGE_FTDI_RESET_CONTROL) {
            configuredFtdiReset = true;
            discoverStartTime = 0;
            RunSynchronousSteps();
            return;
        } else if (PacketIsQueryInterface(packetId)) {
            if (!packetInterfaceId.has_value()) {
                packetInterfaceId = ReadUint16(payload);
                discoverStartTime = 0;
                RunSynchronousSteps();
            }
            return;
        }

        // Anything else will adjust the state advance

        switch (packetReferenceNumber) {
            case MESSAGE_BULK_INPUT: {
                if (!ntStore) {
                    break;
                }

                double motor0Reversed =
                    ntStore->motors[0].reversedSubscriber.Get(false) ? 1.0
                                                                     : -1.0;
                double motor1Reversed =
                    ntStore->motors[1].reversedSubscriber.Get(false) ? 1.0
                                                                     : -1.0;
                double motor2Reversed =
                    ntStore->motors[2].reversedSubscriber.Get(false) ? 1.0
                                                                     : -1.0;
                double motor3Reversed =
                    ntStore->motors[3].reversedSubscriber.Get(false) ? 1.0
                                                                     : -1.0;

                ntStore->motors[0].encoderPublisher.Set(
                    ReadInt32(payload.subspan(1)) * motor0Reversed);
                ntStore->motors[1].encoderPublisher.Set(
                    ReadInt32(payload.subspan(5)) * motor1Reversed);
                ntStore->motors[2].encoderPublisher.Set(
                    ReadInt32(payload.subspan(9)) * motor2Reversed);
                ntStore->motors[3].encoderPublisher.Set(
                    ReadInt32(payload.subspan(13)) * motor3Reversed);

                ntStore->motors[0].velocityPublisher.Set(
                    ReadInt16(payload.subspan(18)) * motor0Reversed);
                ntStore->motors[1].velocityPublisher.Set(
                    ReadInt16(payload.subspan(20)) * motor1Reversed);
                ntStore->motors[2].velocityPublisher.Set(
                    ReadInt16(payload.subspan(22)) * motor2Reversed);
                ntStore->motors[3].velocityPublisher.Set(
                    ReadInt16(payload.subspan(24)) * motor3Reversed);

                break;
            }
            case MESSAGE_BATTERY_VOLTAGE:
                if (!ntStore) {
                    break;
                }

                ntStore->batteryVoltagePublisher.Set(ReadInt16(payload) /
                                                     1000.0);

                break;

            case MESSAGE_MOTOR_GET_CURRENT_0:
                if (!ntStore) {
                    break;
                }

                ntStore->motors[0].currentPublisher.Set(ReadInt16(payload) /
                                                        1000.0);
                break;

            case MESSAGE_MOTOR_GET_CURRENT_1:
                if (!ntStore) {
                    break;
                }

                ntStore->motors[1].currentPublisher.Set(ReadInt16(payload) /
                                                        1000.0);
                break;

            case MESSAGE_MOTOR_GET_CURRENT_2:
                if (!ntStore) {
                    break;
                }

                ntStore->motors[2].currentPublisher.Set(ReadInt16(payload) /
                                                        1000.0);
                break;

            case MESSAGE_MOTOR_GET_CURRENT_3:
                if (!ntStore) {
                    break;
                }

                ntStore->motors[3].currentPublisher.Set(ReadInt16(payload) /
                                                        1000.0);
                break;

            default:
                // printf("Unknown message number\n");
                break;
        }

        CheckForStateAdvance(static_cast<MessageNumbers>(packetReferenceNumber),
                             data.size());
    }

    void recover() {
        stateMachine.Reset();
        outstandingMessages = 0;
        writeBuffer.clear();
        pendingWrites.clear();
        currentCount = 0;
        receivedCount = 0;
    }

    UvSerial(UvSerial&) = delete;
    UvSerial(UvSerial&&) = delete;
    UvSerial& operator=(UvSerial&) = delete;
    UvSerial& operator=(UvSerial&&) = delete;

    uint8_t readBuf[256];
    int serialFd{-1};
    std::weak_ptr<wpi::uv::Poll> serialPoll;
    eh::ReceiveStateMachine stateMachine{
        [this](auto data, auto crc) { HandlePayload(data, crc); }};

    uint64_t discoverStartTime{0};
    std::optional<uint8_t> address{};
    std::optional<uint16_t> packetInterfaceId{};

    uint8_t outstandingMessages{0};
    std::vector<uint8_t> writeBuffer;
    std::deque<size_t> pendingWrites;
    size_t currentCount{0};
    size_t receivedCount{0};

    uint8_t txBuffer[1024];

    std::string serialPath;
    uint64_t lastLoop;
    bool canEnable{false};

    bool configuredFtdiReset{false};

    SendState sendState = SendState::ReadyToSend;
};
}  // namespace eh

struct ExpansionHubState {
    uint64_t lastLoop = wpi::Now();

    int socketHandle{-1};

    eh::NetworkTablesStore ntStore;

    const nt::NetworkTableInstance* ntInstance;

    unsigned busId{0};

    std::unique_ptr<eh::UvSerial> currentHub;

    ~ExpansionHubState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void onUpdate(bool canEnable);

    void sendInitial();

    void sendCommands(bool canEnable);

    bool startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                     wpi::uv::Loop& loop);

    void onDeviceAdded(std::unique_ptr<eh::UvSerial> hub);

    void onDeviceRemoved(std::string_view port);
};

void ExpansionHubState::sendInitial() {
    // Make sure keep alive is the first thing sent, its needed for recovery
    currentHub->SendKeepAlive();
    currentHub->GetModuleStatus();

    // Do requests next
    currentHub->SendBatteryRequest();
    currentHub->SendBulkInput();

    // Then the servos
    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        currentHub->SendServoPulseWidth(
            i, ntStore.servos[i].pulseWidthSubscriber.Get(1500));
    }

    currentHub->Flush();
}

void ExpansionHubState::sendCommands(bool canEnable) {
    // We've unrolled these so we can control updates together to make
    // sure they happen as close as possible.

    // First the 4 motors, as those will get sent in the same UART request as
    // the initial send.
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        double reversed =
            ntStore.motors[i].reversedSubscriber.Get(false) ? 1.0 : -1.0;
        currentHub->SendMotorConstantPower(
            i, ntStore.motors[i].setpointSubscriber.Get(0) * reversed);
    }

    // Then the motor currents
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        currentHub->SendMotorCurrentRequest(i);
    }

    // Then all the things that are not expect to change, but we want to keep
    // sending in case of reset.
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        currentHub->SendMotorMode(
            i, ntStore.motors[i].floatOn0Subscriber.Get(false));
        currentHub->SendMotorEnable(
            i,
            canEnable ? ntStore.motors[i].enabledSubscriber.Get(false) : false);
    }

    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        currentHub->SendServoConfiguration(
            i, ntStore.servos[i].framePeriodSubscriber.Get(20000));

        currentHub->SendServoEnable(
            i,
            canEnable ? ntStore.servos[i].enabledSubscriber.Get(false) : false);
    }

    currentHub->Flush();
}

void ExpansionHubState::onDeviceAdded(std::unique_ptr<eh::UvSerial> hub) {
    currentHub = std::move(hub);

    ntStore.Initialize(*ntInstance, busId);

    ntStore.isConnectedPublisher.Set(true);
    printf("Device added\n");
    ntStore.numNacks = 0;
    ntStore.numNacksPublisher.Set(ntStore.numNacks);
    ntStore.numCrcFailures = 0;
    ntStore.numCrcFailuresPublisher.Set(ntStore.numCrcFailures);
    ntStore.numMissedSendLoops = 0;
    ntStore.numMissedSendLoopsPublisher.Set(ntStore.numMissedSendLoops);
    currentHub->SetCallbacks(
        [this](bool canEnable) { sendCommands(canEnable); }, &ntStore);

    // TODO we only want the timer running if we have a device.
}

void ExpansionHubState::onDeviceRemoved(std::string_view path) {
    if (currentHub && path == currentHub->serialPath) {
        ntStore.isConnectedPublisher.Set(false);
        currentHub.reset();
    }
}

void ExpansionHubState::onUpdate(bool canEnable) {
    if (!currentHub) {
        return;
    }

    if (!currentHub->HasFinishedSynchronous()) {
        currentHub->RunSynchronousSteps();
        return;
    }

    auto now = wpi::Now();
    auto delta = now - lastLoop;

    bool allowSend = currentHub->AllowSend();

    if (!allowSend && delta < 1000000) {
        printf("Skipping due to outstanding\n");
        ntStore.numMissedSendLoops++;
        ntStore.numMissedSendLoopsPublisher.Set(ntStore.numMissedSendLoops);
        return;
    } else if (!allowSend && delta >= 1000000) {
        printf("1 second timeout. Attempting to recover %d %d %d\n",
               (int)currentHub->outstandingMessages,
               (int)currentHub->currentCount, (int)currentHub->receivedCount);
        currentHub->recover();
    }

    lastLoop = now;

    if (delta > 23000) {
        printf("Delta time %lu\n", delta);
    }

    currentHub->StartTransaction(canEnable);

    sendInitial();
}

static void OnDeviceRemoved(
    std::array<ExpansionHubState, NUM_USB_BUSES>& states, sd_device* device) {
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

    std::string_view path = devPath.c_str();

    for (auto&& i : states) {
        i.onDeviceRemoved(path);
    }
}

static void OnDeviceAdded(wpi::uv::Loop& loop,
                          std::array<ExpansionHubState, NUM_USB_BUSES>& states,
                          sd_device* device) {
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

    if (states[busNum].currentHub != nullptr) {
        printf("Received duplicate bus, likely race condition\n");
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

    ret = rhsp_serialOpenDirect(devPath.c_str(), 460800, 8,
                                RHSP_SERIAL_PARITY_NONE, 1,
                                RHSP_SERIAL_FLOW_CONTROL_NONE);
    if (ret < 0) {
        printf("rhsp_serialOpenDirect failed %d\n", ret);
        return;
    }

    std::unique_ptr<eh::UvSerial> sl = std::make_unique<eh::UvSerial>();
    bool isInit = sl->initialize(loop, ret);
    printf("initialized %d\n", isInit ? 1 : 0);

    sl->serialPath = devPath;

    sl->RunSynchronousSteps();

    states[busNum].onDeviceAdded(std::move(sl));
}

bool ExpansionHubState::startUvLoop(unsigned bus,
                                    const nt::NetworkTableInstance& ntInst,
                                    wpi::uv::Loop& loop) {
    if (bus >= NUM_USB_BUSES) {
        return false;
    }

    busId = bus;
    ntInstance = &ntInst;

    return true;
}

int main() {
    printf("Starting ExpansionHubDaemon\n");
    printf("\tBuild Hash: %s\n", MRC_GetGitHash());
    printf("\tBuild Timestamp: %s\n", MRC_GetBuildTimestamp());

#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
#endif

    int retries = 0;
    int control_data_fd = -1;
    while (control_data_fd == -1 && retries < 50) {
        control_data_fd = open(CONTROL_DATA_PATH, O_RDONLY);
        if (control_data_fd == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            retries++;
        }
    }

    if (control_data_fd == -1) {
        printf("Failed to open control data.\n");
        return -1;
    }

    std::array<ExpansionHubState, NUM_USB_BUSES> states;

    auto ntInst = nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient("ExpansionHubDaemon");

    wpi::EventLoopRunner loopRunner;

    struct LoopStorage {
        std::array<ExpansionHubState, NUM_USB_BUSES>* hubStates;
        wpi::uv::Loop* loop;
    } loopStorage{
        .hubStates = &states,
        .loop = nullptr,
    };

    bool success = false;
    loopRunner.ExecSync([&success, &states, &loopStorage, &ntInst,
                         control_data_fd](wpi::uv::Loop& loop) {
        loopStorage.loop = &loop;
        for (size_t i = 0; i < states.size(); i++) {
            success = states[i].startUvLoop(i, ntInst, loop);
            if (!success) {
                return;
            }
        }

        sd_device_monitor* devMonitor = nullptr;
        int res = sd_device_monitor_new(&devMonitor);

        if (res < 0) {
            printf("sd_device_monitor_new failed %d\n", res);
            success = false;
            return;
        }

        res = sd_device_monitor_filter_add_match_subsystem_devtype(
            devMonitor, "usb-serial", NULL);
        if (res < 0) {
            printf(
                "sd_device_monitor_filter_add_match_subsystem_devtype failed "
                "%d\n",
                res);
            success = false;
            return;
        }

        res = sd_device_monitor_start(
            devMonitor,
            [](sd_device_monitor* m, sd_device* device, void* userdata) {
                sd_device_action_t action = _SD_DEVICE_ACTION_INVALID;
                sd_device_get_action(device, &action);

                if (action == SD_DEVICE_BIND) {
                    printf("Detected new device\n");
                    LoopStorage* ls = reinterpret_cast<LoopStorage*>(userdata);
                    OnDeviceAdded(*ls->loop, *ls->hubStates, device);
                } else if (action == SD_DEVICE_REMOVE) {
                    LoopStorage* ls = reinterpret_cast<LoopStorage*>(userdata);
                    OnDeviceRemoved(*ls->hubStates, device);
                }

                return 0;
            },
            &loopStorage);

        if (res < 0) {
            printf("sd_device_monitor_start failed %d\n", res);
            success = false;
            return;
        }

        sd_event* sdEvent = sd_device_monitor_get_event(devMonitor);
        if (!sdEvent) {
            printf("sd_device_monitor_get_event failed\n");
            success = false;
            return;
        }

        int fd = sd_event_get_fd(sdEvent);
        if (fd < 0) {
            printf("sd_event_get_fd failed %d\n", fd);
            success = false;
            return;
        }

        auto poll = wpi::uv::Poll::Create(loop, fd);
        if (!poll) {
            printf("Poll create failed\n");
            success = false;
            return;
        }

        poll->pollEvent.connect(
            [sdEvent](int flags) { sd_event_run(sdEvent, 0); });

        poll->Start(UV_READABLE);

        auto sendTimer = wpi::uv::Timer::Create(loop);

        sendTimer->timeout.connect([&states, control_data_fd]() {
            char buf[128];

            struct robot_state state;
            memset(&state, 0, sizeof(state));

            ssize_t control_data_size =
                pread(control_data_fd, buf, sizeof(buf), 0);

            buf[control_data_size] = '\0';

            unsigned int control_data = strtol(buf, NULL, 16);

            bool system_watchdog = (control_data & 0x1) != 0 ? true : false;

            for (auto&& dev : states) {
                dev.onUpdate(system_watchdog);
            }
        });

        sendTimer->Start(wpi::uv::Timer::Time{20}, wpi::uv::Timer::Time{20});

        // Enumerate everything for initial checking

        sd_device_enumerator* devEnum;
        sd_device_enumerator_new(&devEnum);
        sd_device_enumerator_add_match_subsystem(devEnum, "usb-serial", 1);

        sd_device* newDev = sd_device_enumerator_get_device_first(devEnum);
        while (newDev) {
            const char* path = "UNKNOWN";
            sd_device_get_subsystem(newDev, &path);
            if (strcmp(path, "usb-serial") == 0) {
                OnDeviceAdded(loop, states, newDev);
            }
            newDev = sd_device_enumerator_get_device_next(devEnum);
        }
    });

    if (!success) {
        loopRunner.Stop();
        close(control_data_fd);
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

    loopRunner.Stop();
    close(control_data_fd);

    return 0;
}
