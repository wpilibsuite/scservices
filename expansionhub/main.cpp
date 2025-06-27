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

#define NUM_USB_BUSES 4
#define NUM_MOTORS_PER_HUB 4
#define NUM_SERVOS_PER_HUB 6
#define MAX_NUM_OUTSTANDING_MESSAGES 8

#define RHSP_ARRAY_DWORD(type, buffer, startIndex)                          \
    ((type)(buffer)[startIndex] | ((type)(buffer)[(startIndex) + 1] << 8) | \
     ((type)(buffer)[(startIndex) + 2] << 16) |                             \
     ((type)(buffer)[(startIndex) + 3] << 24))

#define RHSP_ARRAY_WORD(type, buffer, startIndex) \
    ((type)(buffer)[startIndex] | ((type)(buffer)[(startIndex) + 1] << 8))

namespace eh {

static uint8_t calcChecksum(const uint8_t* buffer, size_t bufferSize) {
    uint8_t sum = 0;
    for (size_t i = 0; i < bufferSize; i++) {
        sum += buffer[i];
    }
    return sum;
}

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

    void SetCallbacks(
        std::function<void(std::span<const int32_t>, std::span<const int16_t>)>
            onEncoders,
        std::function<void(double)> onBatteryVoltage) {
        _onEncoders = std::move(onEncoders);
        _onBatteryVoltage = std::move(onBatteryVoltage);
    }

    std::function<void(std::span<const int32_t>, std::span<const int16_t>)>
        _onEncoders;

    std::function<void(double)> _onBatteryVoltage;

    void RunDiscoverInternal() {
        if (address.has_value()) {
            return;
        }

        auto now = wpi::Now();
        auto delta = now - discoverStartTime;

        // Don't try again
        if (delta <= 1000000) {
            return;
        }

        discoverStartTime = now;

        SendPacket(0xFF, MESSAGE_DISCOVER, 0x7F0F, {}, true);
    }

    void RunInterfacePacketId() {
        if (packetInterfaceId.has_value()) {
            return;
        }

        if (!address.has_value()) {
            RunDiscoverInternal();
            return;
        }

        auto now = wpi::Now();
        auto delta = now - discoverStartTime;

        // Don't try again
        if (delta <= 1000000) {
            return;
        }

        discoverStartTime = now;

        SendPacket(*address, MESSAGE_QUERY_INTERFACE, 0x7F07,
                   std::span<const uint8_t>{
                       reinterpret_cast<const uint8_t*>("DEKA"), 5},
                   true);
    }

    void SendBatteryRequest() {
        uint16_t packetId = *packetInterfaceId + 7;
        uint8_t buffer[2] = {13, 0};
        SendPacket(*address, MESSAGE_BATTERY_VOLTAGE, packetId, buffer);
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

#define POWER_CONVERSION 32767

    void SendMotorConstantPower(uint8_t channel, double power) {
        power = std::clamp(power, -1.0, 1.0);

        uint16_t packetId = *packetInterfaceId + 15;

        int16_t adjustedPowerLevel = (int16_t)(power * POWER_CONVERSION);

        uint8_t buffer[3] = {channel, (uint8_t)(adjustedPowerLevel),
                             (uint8_t)(adjustedPowerLevel >> 8)};

        SendPacket(*address, MESSAGE_MOTOR_SET_CONSTANT_POWER_0 + channel,
                   packetId, buffer);
    }

    void SendMotorMode(uint8_t channel) {
        uint16_t packetId = *packetInterfaceId + 8;
        uint8_t cmdPayload[3] = {channel, 0, 0};
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
        SendPacket(*address, MESSAGE_MODULE_STATUS, 0x7F03,
                   std::span<const uint8_t>{&clear, 1});
    }

    void SendKeepAlive() {
        SendPacket(*address, MESSAGE_KEEP_ALIVE, 0x7F04, {});
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
        txBuffer[0] = 0x44;
        txBuffer[1] = 0x4B;
        // TODO bounds check payload
        uint16_t bytesToSend = 10 + payload.size() + 1;
        // @TODO implement function to make uint16 from two uint8 values
        txBuffer[2] = (uint8_t)bytesToSend;
        txBuffer[3] = (uint8_t)(bytesToSend >> 8);
        txBuffer[4] = destAddr;
        txBuffer[5] = 0x00;
        txBuffer[6] = messageNumber;
        txBuffer[7] = 0x00;
        // @TODO implement function to make uint16 from two uint8 values
        txBuffer[8] = (uint8_t)packetTypeId;
        txBuffer[9] = (uint8_t)(packetTypeId >> 8);
        if (!payload.empty()) {
            memcpy(&txBuffer[10], payload.data(), payload.size());
        }
        txBuffer[10 + payload.size()] =
            calcChecksum(txBuffer, 10 + payload.size());

        if (direct) {
            write(serialFd, txBuffer, bytesToSend);
        } else {
            writeBuffer.insert(writeBuffer.end(), txBuffer,
                               txBuffer + bytesToSend);
            pendingWrites.emplace_back(bytesToSend);
        }

        // ssize_t written =
        // write(serialFd, txBuffer, bytesToSend);
        // printf("Written %ld\n", written);
    }

    void DoRead() {
        ssize_t readVal = read(serialFd, readBuf, sizeof(readBuf));
        if (readVal <= 0) {
            // Error, do something.
            printf("Read error\n");
            return;
        }

        // printf("Received %ld\n", readVal);

        // Send bytes up
        stateMachine.HandleBytes(
            std::span<const uint8_t>{readBuf, static_cast<size_t>(readVal)});
        Flush();
    }

    bool AllowSend() {
        return pendingWrites.empty() && outstandingMessages == 0;
    }

    void StartTransaction() {
        writeBuffer.clear();
        currentCount = 0;
        receivedCount = 0;
        lastLoop = wpi::Now();
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
            if (AllowSend()) {
                auto delta = wpi::Now() - lastLoop;
                printf("Time to finish %ld %ld %ld\n", delta, currentCount,
                       receivedCount);
            }
            return;
        }

        //printf("Writing %d of %d messages\n", (int)toWrite, (int)available);

        write(serialFd, writeBuffer.data() + currentCount, count);
        // printf("Written %ld\n", w);
        outstandingMessages += toWrite;
        currentCount += count;
    }

    void HandlePayload(std::span<const uint8_t> data) {
        uint16_t packetId =
            ((uint16_t)(data[9]) << 8 | (uint16_t)(data[8])) & ~0x8000;
        uint8_t sentMessageId = data[7];

        receivedCount += data.size();

        auto payload = data.subspan(10, data.size() - (11));

        if (sentMessageId != MESSAGE_DISCOVER &&
            sentMessageId != MESSAGE_QUERY_INTERFACE) {
            outstandingMessages--;
        }

        if (packetId == 0x7F02) {
            printf("Nack %d for message id %d\n", data[10], sentMessageId);
            return;
        }

        switch (sentMessageId) {
            case MESSAGE_DISCOVER:
                if (!address.has_value() && data[10] == 1) {
                    address = data[5];
                    discoverStartTime = 0;
                    RunInterfacePacketId();
                }
                break;
            case MESSAGE_QUERY_INTERFACE:
                if (!packetInterfaceId.has_value()) {
                    packetInterfaceId =
                        ((uint16_t)(data[11]) << 8 | (uint16_t)(data[10]));
                    printf("Packet interface %x, ready to send\n",
                           *packetInterfaceId);
                }
                break;

                // TODO servos

            case MESSAGE_BULK_INPUT: {
                int32_t encoders[4];
                int16_t encoderVels[4];

                encoders[0] = RHSP_ARRAY_DWORD(int32_t, payload.data(), 1);
                encoders[1] = RHSP_ARRAY_DWORD(int32_t, payload.data(), 5);
                encoders[2] = RHSP_ARRAY_DWORD(int32_t, payload.data(), 9);
                encoders[3] = RHSP_ARRAY_DWORD(int32_t, payload.data(), 13);

                encoderVels[0] = RHSP_ARRAY_WORD(int16_t, payload.data(), 18);
                encoderVels[1] = RHSP_ARRAY_WORD(int16_t, payload.data(), 20);
                encoderVels[2] = RHSP_ARRAY_WORD(int16_t, payload.data(), 22);
                encoderVels[3] = RHSP_ARRAY_WORD(int16_t, payload.data(), 24);

                if (_onEncoders) {
                    _onEncoders(encoders, encoderVels);
                }
                break;
            }
            case MESSAGE_KEEP_ALIVE:

                break;

            case MESSAGE_BATTERY_VOLTAGE:

                if (_onBatteryVoltage) {
                    int16_t adc = RHSP_ARRAY_WORD(int32_t, payload.data(), 0);
                    _onBatteryVoltage(adc / 1000.0);
                }

                break;
            default:
                // printf("Unknown message number\n");
                break;
        }

        // printf("Received message id %x %x %x %x %x %x 0x%x %d\n", data[4],
        //        data[5], data[6], data[7], data[8], data[9], packetId,
        //        (int)data.size());
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
        [this](auto data) { HandlePayload(data); }};

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
};
}  // namespace eh

struct ExpansionHubState {
    uint64_t lastLoop = wpi::Now();

    int socketHandle{-1};
    std::array<nt::IntegerPublisher, NUM_MOTORS_PER_HUB> encoderPublishers;
    std::array<nt::IntegerPublisher, NUM_MOTORS_PER_HUB>
        encoderVelocityPublishers;
    std::array<nt::DoubleSubscriber, NUM_MOTORS_PER_HUB> motorPowerSubscribers;

    std::array<nt::BooleanSubscriber, NUM_SERVOS_PER_HUB>
        servoEnabledSubscribers;
    std::array<nt::IntegerSubscriber, NUM_SERVOS_PER_HUB>
        servoPulseWidthSubscribers;
    std::array<nt::IntegerSubscriber, NUM_SERVOS_PER_HUB>
        servoFramePeriodSubscribers;

    nt::DoublePublisher batteryVoltage;

    nt::BooleanPublisher isConnected;

    unsigned busId{0};

    std::unique_ptr<eh::UvSerial> currentHub;

    ~ExpansionHubState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void onUpdate();

    bool startUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                     wpi::uv::Loop& loop);

    void onDeviceAdded(std::unique_ptr<eh::UvSerial> hub);

    void onDeviceRemoved(std::string_view port);
};

void ExpansionHubState::onDeviceAdded(std::unique_ptr<eh::UvSerial> hub) {
    currentHub = std::move(hub);
    isConnected.Set(true);

    currentHub->SetCallbacks(
        [this](auto encoders, auto velocities) {
            for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
                encoderPublishers[i].Set(encoders[i]);
                encoderVelocityPublishers[i].Set(velocities[i]);
            }
        },
        [this](double voltage) { batteryVoltage.Set(voltage); });
    // TODO we only want the timer running if we have a device.
}

void ExpansionHubState::onDeviceRemoved(std::string_view path) {
    if (currentHub && path == currentHub->serialPath) {
        isConnected.Set(false);
        currentHub.reset();
    }
}

void ExpansionHubState::onUpdate() {
    if (!currentHub) {
        return;
    }

    if (!currentHub->packetInterfaceId.has_value()) {
        currentHub->RunInterfacePacketId();
        return;
    }

    auto now = wpi::Now();
    auto delta = now - lastLoop;

    bool allowSend = currentHub->AllowSend();

    if (!allowSend && delta < 1000000) {
        printf("Skipping due to outstanding\n");
        return;
    } else if (!allowSend && delta >= 1000000) {
        printf("1 second timeout. Attempting to recover %d %d %d\n", (int)currentHub->outstandingMessages, (int)currentHub->currentCount, (int)currentHub->receivedCount);
        currentHub->recover();
    }

    lastLoop = now;

    printf("Delta time %lu\n", delta);

    currentHub->StartTransaction();

    // Make sure keep alive is the first thing sent, its needed for recovery
    currentHub->SendKeepAlive();
    currentHub->GetModuleStatus();

    currentHub->SendBatteryRequest();
    currentHub->SendBulkInput();


    // We also need to handle disconnect.
    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        currentHub->SendServoConfiguration(
            i, servoFramePeriodSubscribers[i].Get(20000));

        currentHub->SendServoPulseWidth(i,
                                        servoPulseWidthSubscribers[i].Get(1500));

        currentHub->SendServoEnable(i, servoEnabledSubscribers[i].Get(false));
    }

    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        currentHub->SendMotorMode(i);
        currentHub->SendMotorConstantPower(i, motorPowerSubscribers[i].Get(0));
        currentHub->SendMotorEnable(i, true);
    }

    currentHub->Flush();
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

    sl->RunInterfacePacketId();

    states[busNum].onDeviceAdded(std::move(sl));
}

bool ExpansionHubState::startUvLoop(unsigned bus,
                                    const nt::NetworkTableInstance& ntInst,
                                    wpi::uv::Loop& loop) {
    if (bus >= NUM_USB_BUSES) {
        return false;
    }

    busId = bus;

    nt::PubSubOptions options;
    options.sendAll = true;
    options.keepDuplicates = true;
    options.periodic = 0.005;

    auto busIdStr = std::to_string(busId);

    batteryVoltage = ntInst.GetDoubleTopic("/rhsp/" + busIdStr + "/battery")
                         .Publish(options);
    isConnected = ntInst.GetBooleanTopic("/rhsp/" + busIdStr + "/connected")
                      .Publish(options);

    for (size_t i = 0; i < encoderPublishers.size(); i++) {
        auto iStr = std::to_string(i);
        encoderPublishers[i] =
            ntInst
                .GetIntegerTopic("/rhsp/" + busIdStr + "/motor" + iStr +
                                 "/encoder")
                .Publish(options);
        encoderVelocityPublishers[i] =
            ntInst
                .GetIntegerTopic("/rhsp/" + busIdStr + "/motor" + iStr +
                                 "/encoderVelocity")
                .Publish(options);
        motorPowerSubscribers[i] =
            ntInst
                .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + iStr +
                                "/power")
                .Subscribe(0, options);
    }

    for (size_t i = 0; i < servoEnabledSubscribers.size(); i++) {
        auto iStr = std::to_string(i);
        servoEnabledSubscribers[i] =
            ntInst
                .GetBooleanTopic("/rhsp/" + busIdStr + "/servo" + iStr +
                                 "/enabled")
                .Subscribe(false, options);
        servoFramePeriodSubscribers[i] =
            ntInst
                .GetIntegerTopic("/rhsp/" + busIdStr + "/servo" + iStr +
                                 "/framePeriod")
                .Subscribe(0, options);
        servoPulseWidthSubscribers[i] =
            ntInst
                .GetIntegerTopic("/rhsp/" + busIdStr + "/servo" + iStr +
                                 "/pulseWidth")
                .Subscribe(0, options);
    }

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
    loopRunner.ExecSync([&success, &states, &loopStorage,
                         &ntInst](wpi::uv::Loop& loop) {
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

        sendTimer->timeout.connect([&states]() {
            // printf("send?\n");
            for (auto&& dev : states) {
                dev.onUpdate();
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
