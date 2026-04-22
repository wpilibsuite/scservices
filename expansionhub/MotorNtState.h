#pragma once

#include "wpi/nt/BooleanTopic.hpp"
#include "wpi/nt/IntegerTopic.hpp"
#include "wpi/nt/DoubleTopic.hpp"
#include "PositionController.h"
#include "VelocityController.h"
#include "CachedCommand.h"
#include <utility>

namespace eh {

struct MotorNtState {
    CachedCommand<wpi::nt::BooleanSubscriber> enabledSubscriber;

    wpi::nt::DoublePublisher encoderPublisher;
    wpi::nt::DoublePublisher velocityPublisher;
    wpi::nt::DoublePublisher currentPublisher;

    CachedCommand<wpi::nt::BooleanSubscriber> floatOn0Subscriber;
    wpi::nt::IntegerSubscriber modeSubscriber;

    wpi::nt::DoubleSubscriber setpointSubscriber;

    PositionController positionPid;
    VelocityController velocityPid;

    wpi::nt::BooleanSubscriber reversedSubscriber;
    wpi::nt::BooleanSubscriber resetEncoderSubscriber;

    wpi::nt::DoubleSubscriber distancePerCountSubscriber;

    bool doReset{false};
    int64_t lastResetTime{0};

    void Initialize(const wpi::nt::NetworkTableInstance& instance, int motorNum,
                    const std::string& busIdStr, wpi::nt::PubSubOptions options);

    double lastEncoderPosition{0};
    double lastEncoderVelocity{0};

    std::pair<double, int> ComputeMotorPower(double batteryVoltage);

    void SetEncoder(double positionRaw, double velocityRaw);
};

}  // namespace eh
