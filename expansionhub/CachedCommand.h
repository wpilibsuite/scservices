#pragma once

#include <optional>

namespace eh {

template <typename T>
struct CachedCommand {
    T subscriber;
    std::optional<typename T::ValueType> lastAckedValue;
    T::ValueType lastAttemptedValue;

    CachedCommand() = default;

    CachedCommand(const CachedCommand&) = delete;
    CachedCommand& operator=(const CachedCommand&) = delete;
    CachedCommand(CachedCommand&&) = delete;
    CachedCommand& operator=(CachedCommand&&) = delete;

    CachedCommand& operator=(T sub) {
        subscriber = std::move(sub);
        return *this;
    }

    void ForceReset() { lastAckedValue.reset(); }

    void Ack() { lastAckedValue = lastAttemptedValue; }

    std::optional<typename T::ValueType> Get() {
        auto newValue = subscriber.Get();

        if (newValue != lastAckedValue) {
            lastAckedValue.reset();
            lastAttemptedValue = newValue;
            return newValue;
        }

        return std::nullopt;
    }

    std::optional<typename T::ValueType> GetWithCanEnable(bool canEnable) {
        auto newValue = canEnable ? subscriber.Get() : false;

        if (newValue != lastAckedValue) {
            lastAckedValue.reset();
            lastAttemptedValue = newValue;
            return newValue;
        }

        return std::nullopt;
    }
};
}  // namespace eh
