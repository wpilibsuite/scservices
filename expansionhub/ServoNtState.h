#pragma once

#include "wpi/nt/BooleanTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"
#include "CachedCommand.h"
#include <string>

namespace eh {

struct ServoNtState {
    CachedCommand<wpi::nt::BooleanSubscriber> enabledSubscriber;
    wpi::nt::IntegerSubscriber pulseWidthSubscriber;
    CachedCommand<wpi::nt::IntegerSubscriber> framePeriodSubscriber;

    void Initialize(const wpi::nt::NetworkTableInstance& instance, int servoNum,
                    const std::string& busIdStr, wpi::nt::PubSubOptions options);
};

}
