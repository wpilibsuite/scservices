#pragma once

#include "networktables/BooleanTopic.h"
#include "networktables/IntegerTopic.h"
#include "networktables/DoubleTopic.h"
#include "PidConstants.h"
#include "CachedCommand.h"

namespace eh {

struct MotorNtState {
    CachedCommand<nt::BooleanSubscriber> enabledSubscriber;

    nt::DoublePublisher encoderPublisher;
    nt::DoublePublisher velocityPublisher;
    nt::DoublePublisher currentPublisher;

    CachedCommand<nt::BooleanSubscriber> floatOn0Subscriber;
    nt::IntegerSubscriber modeSubscriber;

    nt::DoubleSubscriber setpointSubscriber;

    PidConstants positionPid;
    PidConstants velocityPid;

    nt::BooleanSubscriber reversedSubscriber;
    nt::BooleanSubscriber resetEncoderSubscriber;

    nt::DoubleSubscriber distancePerCountSubscriber;

    bool doReset{false};
    int64_t lastResetTime{0};

    void Initialize(const nt::NetworkTableInstance& instance, int motorNum,
                    const std::string& busIdStr, nt::PubSubOptions options);

    double lastEncoderPosition{0};
    double lastEncoderVelocity{0};

    double ComputeMotorPower(double batteryVoltage);

    void SetEncoder(double positionRaw, double velocityRaw);
};

}  // namespace eh
