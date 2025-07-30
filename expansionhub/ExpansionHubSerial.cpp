#include "ExpansionHubSerial.h"

#include "stdio.h"

#include "wpi/timestamp.h"

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

using namespace eh;

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

bool ExpansionHubSerial::Initialize(wpi::uv::Loop& loop, int fd, std::string path) {
    serialFd = fd;
    serialPath = std::move(path);
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

void ExpansionHubSerial::SetCallbacks(std::function<void(bool, bool)> doOnSendCommands,
                  ExpansionHubNtState* store) {
    onSendCommands = std::move(doOnSendCommands), ntStore = store;
}

void ExpansionHubSerial::RunDiscoverInternal() {
    auto now = wpi::Now();
    auto delta = now - discoverStartTime;

    // Don't try again
    if (delta <= MESSAGE_TIMEOUT) {
        return;
    }

    discoverStartTime = now;

    SendPacket(DISCOVER_ADDRESS, MESSAGE_DISCOVER, DISCOVER_ID, {}, true);
}

void ExpansionHubSerial::RunInterfacePacketIdInternal() {
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

void ExpansionHubSerial::RunFtdiConfigureInternal() {
    auto now = wpi::Now();
    auto delta = now - discoverStartTime;

    // Don't try again
    if (delta <= MESSAGE_TIMEOUT) {
        return;
    }

    discoverStartTime = now;
    uint16_t packetId = *packetInterfaceId + 49;
    uint8_t buffer[1] = {1};

    SendPacket(*address, MESSAGE_FTDI_RESET_CONTROL, packetId, buffer, true);
}

void ExpansionHubSerial::RunSynchronousSteps() {
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

bool ExpansionHubSerial::HasFinishedSynchronous() {
    return configuredFtdiReset;
}

void ExpansionHubSerial::SendBatteryRequest() {
    uint16_t packetId = *packetInterfaceId + 7;
    uint8_t buffer[2] = {BATTERY_ADC, 0};
    SendPacket(*address, MESSAGE_BATTERY_VOLTAGE, packetId, buffer);
}

void ExpansionHubSerial::SendMotorCurrentRequest(uint8_t channel) {
    uint16_t packetId = *packetInterfaceId + 7;
    uint8_t adcChannel = MOTOR_0_ADC + channel;
    uint8_t buffer[2] = {adcChannel, 0};
    SendPacket(*address, MESSAGE_MOTOR_GET_CURRENT_0 + channel, packetId,
               buffer);
}

void ExpansionHubSerial::SendEncoderResetRequest(uint8_t channel) {
    uint16_t packetId = *packetInterfaceId + 14;
    uint8_t buffer[1] = {channel};
    SendPacket(*address, MESSAGE_MOTOR_RESET_ENCODER_0 + channel, packetId,
               buffer);
}

void ExpansionHubSerial::SendServoConfiguration(uint8_t channel, uint16_t framePeriod) {
    if (framePeriod <= 1) {
        return;
    }
    uint16_t packetId = *packetInterfaceId + 31;

    uint8_t buffer[3] = {channel, (uint8_t)(framePeriod),
                         (uint8_t)(framePeriod >> 8)};

    SendPacket(*address, MESSAGE_SERVO_CONFIGURATION_0 + channel, packetId,
               buffer);
}

void ExpansionHubSerial::SendServoPulseWidth(uint8_t channel, uint16_t pulseWidth) {
    if (pulseWidth == 0) {
        return;
    }
    uint16_t packetId = *packetInterfaceId + 33;

    uint8_t buffer[3] = {channel, (uint8_t)(pulseWidth),
                         (uint8_t)(pulseWidth >> 8)};

    SendPacket(*address, MESSAGE_SERVO_PULSE_WIDTH_0 + channel, packetId,
               buffer);
}

void ExpansionHubSerial::SendServoEnable(uint8_t channel, bool enable) {
    uint16_t packetId = *packetInterfaceId + 35;
    uint8_t enableVal = enable ? 1 : 0;
    uint8_t buffer[2] = {channel, enableVal};
    SendPacket(*address, MESSAGE_SERVO_ENABLE_0 + channel, packetId, buffer);
}

void ExpansionHubSerial::SendMotorConstantPower(uint8_t channel, double power) {
    power = std::clamp(power, -1.0, 1.0);

    uint16_t packetId = *packetInterfaceId + 15;

    int16_t adjustedPowerLevel = (int16_t)(power * POWER_CONVERSION);

    uint8_t buffer[3] = {channel, (uint8_t)(adjustedPowerLevel),
                         (uint8_t)(adjustedPowerLevel >> 8)};

    SendPacket(*address, MESSAGE_MOTOR_SET_CONSTANT_POWER_0 + channel, packetId,
               buffer);
}

void ExpansionHubSerial::SendMotorMode(uint8_t channel, bool floatVal) {
    uint16_t packetId = *packetInterfaceId + 8;
    uint8_t doFloat = floatVal ? 1 : 0;
    uint8_t cmdPayload[3] = {channel, 0, doFloat};
    SendPacket(*address, MESSAGE_MOTOR_SET_RUN_MODE_0 + channel, packetId,
               cmdPayload);
}

void ExpansionHubSerial::SendMotorEnable(uint8_t channel, bool enable) {
    uint16_t packetId = *packetInterfaceId + 10;
    uint8_t enableVal = enable ? 1 : 0;
    uint8_t buffer[2] = {channel, enableVal};
    SendPacket(*address, MESSAGE_MOTOR_ENABLE_0 + channel, packetId, buffer);
}

void ExpansionHubSerial::GetModuleStatus() {
    uint8_t clear = 1;
    SendPacket(*address, MESSAGE_MODULE_STATUS, MODULE_STATUS_ID,
               std::span<const uint8_t>{&clear, 1});
}

void ExpansionHubSerial::SendKeepAlive() {
    SendPacket(*address, MESSAGE_KEEP_ALIVE, KEEP_ALIVE_ID, {});
}

void ExpansionHubSerial::SendBulkInput() {
    uint16_t packetId = *packetInterfaceId + 0;
    SendPacket(*address, MESSAGE_BULK_INPUT, packetId, {});
}

ExpansionHubSerial::~ExpansionHubSerial() noexcept {
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

void ExpansionHubSerial::SendPacket(uint8_t destAddr, uint8_t messageNumber, uint16_t packetTypeId,
                std::span<const uint8_t> payload, bool direct) {
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

void ExpansionHubSerial::DoRead() {
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

bool ExpansionHubSerial::AllowSend() {
    return sendState == SendState::ReadyToSend;
}

void ExpansionHubSerial::StartTransaction(bool canDoEnable) {
    writeBuffer.clear();
    currentCount = 0;
    lastLoop = wpi::Now();
    canEnable = canDoEnable;
    haveBattery = false;
    haveBulk = false;
    haveModuleStatus = false;
    sendState = SendState::WaitingForPackets;
}

void ExpansionHubSerial::Flush() {
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

void ExpansionHubSerial::CheckForStateAdvance(MessageNumbers messageNumber, size_t dataSize) {
    outstandingMessages--;

    if (sendState == SendState::WaitingForPackets) {
        if (haveBattery && haveBulk && haveModuleStatus) {
            if (onSendCommands) {
                onSendCommands(canEnable, deviceReset);
            }
            sendState = SendState::WaitingForFinish;
        }
    }

    if (pendingWrites.empty() && outstandingMessages == 0) {
        if (sendState != SendState::WaitingForFinish) {
            printf("Commands did not occur this loop\n");
        }
        // Done ready to send
        auto delta = wpi::Now() - lastLoop;
        ntStore->transactionTimePublisher.Set(delta);
        sendState = SendState::ReadyToSend;
    }
}

void ExpansionHubSerial::HandlePayload(std::span<const uint8_t> data, uint8_t crc) {
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
        case MESSAGE_MODULE_STATUS:
            haveModuleStatus = true;
            deviceReset = (payload[0] & 0x01) != 0;

            break;
        case MESSAGE_MOTOR_RESET_ENCODER_0: {
            ntStore->motors[0].doReset = false;

            break;
        }

        case MESSAGE_MOTOR_RESET_ENCODER_1: {
            ntStore->motors[1].doReset = false;

            break;
        }

        case MESSAGE_MOTOR_RESET_ENCODER_2: {
            ntStore->motors[2].doReset = false;

            break;
        }
        case MESSAGE_MOTOR_RESET_ENCODER_3: {
            ntStore->motors[3].doReset = false;

            break;
        }

        case MESSAGE_MOTOR_ENABLE_0:
            ntStore->motors[0].enabledSubscriber.Ack();
            break;

        case MESSAGE_MOTOR_ENABLE_1:
            ntStore->motors[1].enabledSubscriber.Ack();
            break;

        case MESSAGE_MOTOR_ENABLE_2:
            ntStore->motors[2].enabledSubscriber.Ack();
            break;
        case MESSAGE_MOTOR_ENABLE_3:
            ntStore->motors[3].enabledSubscriber.Ack();
            break;

        case MESSAGE_MOTOR_SET_RUN_MODE_0:
            ntStore->motors[0].floatOn0Subscriber.Ack();
            break;

        case MESSAGE_MOTOR_SET_RUN_MODE_1:
            ntStore->motors[1].floatOn0Subscriber.Ack();
            break;

        case MESSAGE_MOTOR_SET_RUN_MODE_2:

            ntStore->motors[2].floatOn0Subscriber.Ack();
            break;

        case MESSAGE_MOTOR_SET_RUN_MODE_3:
            ntStore->motors[3].floatOn0Subscriber.Ack();
            break;

        case MESSAGE_SERVO_CONFIGURATION_0:
            ntStore->servos[0].framePeriodSubscriber.Ack();
            break;

        case MESSAGE_SERVO_CONFIGURATION_1:
            ntStore->servos[1].framePeriodSubscriber.Ack();
            break;

        case MESSAGE_SERVO_CONFIGURATION_2:
            ntStore->servos[2].framePeriodSubscriber.Ack();
            break;

        case MESSAGE_SERVO_CONFIGURATION_3:
            ntStore->servos[3].framePeriodSubscriber.Ack();
            break;

        case MESSAGE_SERVO_CONFIGURATION_4:
            ntStore->servos[4].framePeriodSubscriber.Ack();
            break;

        case MESSAGE_SERVO_CONFIGURATION_5:
            ntStore->servos[5].framePeriodSubscriber.Ack();
            break;

        case MESSAGE_SERVO_ENABLE_0:
            ntStore->servos[0].enabledSubscriber.Ack();
            break;

        case MESSAGE_SERVO_ENABLE_1:
            ntStore->servos[1].enabledSubscriber.Ack();
            break;

        case MESSAGE_SERVO_ENABLE_2:
            ntStore->servos[2].enabledSubscriber.Ack();
            break;

        case MESSAGE_SERVO_ENABLE_3:

            ntStore->servos[3].enabledSubscriber.Ack();
            break;

        case MESSAGE_SERVO_ENABLE_4:
            ntStore->servos[4].enabledSubscriber.Ack();
            break;

        case MESSAGE_SERVO_ENABLE_5:
            ntStore->servos[5].enabledSubscriber.Ack();
            break;

        case MESSAGE_BULK_INPUT: {
            haveBulk = true;

            ntStore->motors[0].SetEncoder(ReadInt32(payload.subspan(1)),
                                          ReadInt16(payload.subspan(18)));
            ntStore->motors[1].SetEncoder(ReadInt32(payload.subspan(5)),
                                          ReadInt16(payload.subspan(20)));
            ntStore->motors[2].SetEncoder(ReadInt32(payload.subspan(9)),
                                          ReadInt16(payload.subspan(22)));
            ntStore->motors[3].SetEncoder(ReadInt32(payload.subspan(13)),
                                          ReadInt16(payload.subspan(24)));

            break;
        }
        case MESSAGE_BATTERY_VOLTAGE: {
            haveBattery = true;

            double battery = ReadInt16(payload) / 1000.0;

            ntStore->lastBattery = battery;

            ntStore->batteryVoltagePublisher.Set(battery);

            break;
        }

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

void ExpansionHubSerial::Recover() {
    stateMachine.Reset();
    outstandingMessages = 0;
    writeBuffer.clear();
    pendingWrites.clear();
    currentCount = 0;
}
