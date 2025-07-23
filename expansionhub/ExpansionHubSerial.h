#pragma once

#include "wpinet/uv/Loop.h"
#include "wpinet/uv/Poll.h"
#include "functional"
#include "ExpansionHubNtState.h"
#include "MessageNumbers.h"
#include "ReceiveStateMachine.h"
#include <deque>

namespace eh {

struct ExpansionHubSerial {
   public:
    enum class SendState {
        ReadyToSend,
        WaitingForPackets,
        WaitingForFinish,
    };

    ExpansionHubSerial() = default;
    ~ExpansionHubSerial() noexcept;

    bool Initialize(wpi::uv::Loop& loop, int fd, std::string path);

    void SetCallbacks(std::function<void(bool, bool)> doOnSendCommands,
                      ExpansionHubNtState* store);

    void RunSynchronousSteps();
    bool HasFinishedSynchronous();

    void StartTransaction(bool canDoEnable);

    void Recover();
    bool AllowSend();

    void Flush();

    std::string_view SerialPath() const { return serialPath; };

    void SendBatteryRequest();
    void SendMotorCurrentRequest(uint8_t channel);
    void SendEncoderResetRequest(uint8_t channel);
    void SendServoConfiguration(uint8_t channel, uint16_t framePeriod);
    void SendServoPulseWidth(uint8_t channel, uint16_t pulseWidth);
    void SendServoEnable(uint8_t channel, bool enable);
    void SendMotorConstantPower(uint8_t channel, double power);
    void SendMotorMode(uint8_t channel, bool floatVal);
    void SendMotorEnable(uint8_t channel, bool enable);
    void GetModuleStatus();
    void SendKeepAlive();
    void SendBulkInput();

   private:
    ExpansionHubNtState* ntStore{nullptr};
    std::function<void(bool, bool)> onSendCommands;

    void RunDiscoverInternal();
    void RunInterfacePacketIdInternal();
    void RunFtdiConfigureInternal();

    void SendPacket(uint8_t destAddr, uint8_t messageNumber,
                    uint16_t packetTypeId, std::span<const uint8_t> payload,
                    bool direct = false);

    void DoRead();

    void CheckForStateAdvance(MessageNumbers messageNumber, size_t dataSize);
    void HandlePayload(std::span<const uint8_t> data, uint8_t crc);

    ExpansionHubSerial(ExpansionHubSerial&) = delete;
    ExpansionHubSerial(ExpansionHubSerial&&) = delete;
    ExpansionHubSerial& operator=(ExpansionHubSerial&) = delete;
    ExpansionHubSerial& operator=(ExpansionHubSerial&&) = delete;

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

    uint8_t txBuffer[1024];

    std::string serialPath;
    uint64_t lastLoop;
    bool canEnable{false};
    bool deviceReset{false};

    bool configuredFtdiReset{false};

    SendState sendState = SendState::ReadyToSend;
    bool haveBulk{false};
    bool haveBattery{false};
    bool haveModuleStatus{false};
};

}  // namespace eh
