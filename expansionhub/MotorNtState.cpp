#include "MotorNtState.h"

#include "networktables/NetworkTableInstance.h"

#define PERCENTAGE_MODE 0
#define VOLTAGE_MODE 1
#define POSITION_PID_MODE 2
#define VELOCITY_PID_MODE 3

using namespace eh;

void MotorNtState::SetEncoder(double positionRaw, double velocityRaw) {
    double reversed = reversedSubscriber.Get(false) ? -1.0 : 1.0;
    double distancePerCount = distancePerCountSubscriber.Get(0);
    if (distancePerCount == 0) {
        distancePerCount = 1;
    }
    lastEncoderPosition = positionRaw * reversed * distancePerCount;
    // TODO does this need to be scaled
    lastEncoderVelocity = velocityRaw * reversed * distancePerCount;

    encoderPublisher.Set(lastEncoderPosition);
    velocityPublisher.Set(lastEncoderVelocity);
}

double MotorNtState::ComputeMotorPower(double batteryVoltage) {
    double reversed = reversedSubscriber.Get(false) ? -1.0 : 1.0;
    if (batteryVoltage == 0) {
        return 0;
    }
    double setpoint = setpointSubscriber.Get(0);
    switch (modeSubscriber.Get(PERCENTAGE_MODE)) {
        case VOLTAGE_MODE:
            return (setpoint / batteryVoltage) * reversed;

        case POSITION_PID_MODE:
            return (positionPid.Compute(setpoint, lastEncoderPosition) /
                    batteryVoltage) *
                   reversed;

        case VELOCITY_PID_MODE:
            return (velocityPid.Compute(setpoint, lastEncoderVelocity) /
                    batteryVoltage) *
                   reversed;

        default:
            return setpoint * reversed;
    }
}

void MotorNtState::Initialize(const nt::NetworkTableInstance& instance,
                              int motorNum, const std::string& busIdStr,
                              nt::PubSubOptions options) {
    auto motorNumStr = std::to_string(motorNum);
    encoderPublisher = instance
                           .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                           motorNumStr + "/encoder")
                           .Publish(options);
    velocityPublisher = instance
                            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                            motorNumStr + "/encoderVelocity")
                            .Publish(options);
    currentPublisher = instance
                           .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                           motorNumStr + "/current")
                           .Publish(options);
    setpointSubscriber = instance
                             .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                             motorNumStr + "/setpoint")
                             .Subscribe(0, options);
    floatOn0Subscriber = instance
                             .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              motorNumStr + "/floatOn0")
                             .Subscribe(false, options);
    enabledSubscriber = instance
                            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                             motorNumStr + "/enabled")
                            .Subscribe(false, options);

    modeSubscriber = instance
                         .GetIntegerTopic("/rhsp/" + busIdStr + "/motor" +
                                          motorNumStr + "/mode")
                         .Subscribe(0, options);

    reversedSubscriber = instance
                             .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              motorNumStr + "/reversed")
                             .Subscribe(false, options);

    distancePerCountSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNumStr +
                            "/distancePerCount")
            .Subscribe(0, options);

    resetEncoderSubscriber =
        instance
            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" + motorNumStr +
                             "/resetEncoder")
            .Subscribe(false, options);

    velocityPid.Initialize(instance, motorNumStr, busIdStr, "velocity",
                           options);
    positionPid.Initialize(instance, motorNumStr, busIdStr, "position",
                           options);
}
