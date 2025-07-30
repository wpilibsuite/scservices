#pragma once

#include "networktables/BooleanTopic.h"
#include "networktables/IntegerTopic.h"
#include "CachedCommand.h"
#include <string>

namespace eh {

struct ServoNtState {
    CachedCommand<nt::BooleanSubscriber> enabledSubscriber;
    nt::IntegerSubscriber pulseWidthSubscriber;
    CachedCommand<nt::IntegerSubscriber> framePeriodSubscriber;

    void Initialize(const nt::NetworkTableInstance& instance, int servoNum,
                    const std::string& busIdStr, nt::PubSubOptions options);
};

}
