#pragma once

#include <functional>
#include <span>
#include <vector>

namespace eh {
class ReceiveStateMachine {
   public:
    ReceiveStateMachine(std::function<void(std::span<const uint8_t>)> onPacket)
        : _onPacket{std::move(onPacket)} {}

    void Reset() {
        _storage.clear();
        _state = CurrentState::FirstByte;
    }

    void HandleBytes(std::span<const uint8_t> buffer) {
        while (!buffer.empty()) {
            size_t bytesToAdvance = 0;
            switch (_state) {
                case CurrentState::FirstByte:
                    bytesToAdvance = 1;
                    if (buffer[0] == 0x44) {
                        _storage.emplace_back(buffer[0]);
                        _state = CurrentState::SecondByte;
                    }
                    break;
                case CurrentState::SecondByte:
                    bytesToAdvance = 1;
                    if (buffer[0] == 0x4B) {
                        _storage.emplace_back(buffer[0]);
                        _state = CurrentState::Header;
                    } else {
                        _storage.clear();
                        _state = CurrentState::FirstByte;
                    }
                    break;
                case CurrentState::Header: {
                    // Copy up to 10
                    size_t numNeeded = 10 - _storage.size();
                    size_t canCopy = (std::min)(numNeeded, buffer.size());
                    bytesToAdvance = canCopy;
                    decltype(buffer.end()) endIterator =
                        buffer.begin() + canCopy;
                    _storage.insert(_storage.end(), buffer.begin(),
                                    endIterator);
                    if (canCopy == numNeeded) {
                        uint16_t packetLength = ((uint16_t)(_storage[3]) << 8 |
                                                 (uint16_t)(_storage[2]));
                        if (packetLength < 11 || packetLength > 1024) {
                            _storage.clear();
                            _state = CurrentState::FirstByte;
                            break;
                        }

                        size_t payloadSize = packetLength - 11;
                        if (payloadSize > 0) {
                            _bytesToReceive = payloadSize;
                            _state = CurrentState::Payload;
                        } else {
                            _state = CurrentState::Crc;
                        }
                    }
                    break;
                }
                case CurrentState::Payload: {
                    // Copy up to payload sized
                    size_t numNeeded = _bytesToReceive - (_storage.size() - 10);
                    size_t canCopy = (std::min)(numNeeded, buffer.size());
                    bytesToAdvance = canCopy;
                    decltype(buffer.end()) endIterator =
                        buffer.begin() + canCopy;
                    _storage.insert(_storage.end(), buffer.begin(),
                                    endIterator);
                    if (canCopy == numNeeded) {
                        _state = CurrentState::Crc;
                    }
                    break;
                }
                case CurrentState::Crc:
                    bytesToAdvance = 1;
                    _state = CurrentState::FirstByte;
                    _storage.emplace_back(buffer[0]);
                    _onPacket(_storage);
                    _storage.clear();
                    break;
            }

            assert(bytesToAdvance != 0);
            buffer = buffer.subspan(bytesToAdvance);
        }
    }

   private:
    enum class CurrentState {
        FirstByte,
        SecondByte,
        Header,
        Payload,
        Crc,
    };

    CurrentState _state{CurrentState::FirstByte};
    size_t _bytesToReceive{0};

    std::vector<uint8_t> _storage;
    std::function<void(std::span<const uint8_t>)> _onPacket;
};
}  // namespace eh
