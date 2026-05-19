#include "PositionController.h"

#include <cmath>
#include <numbers>

#include "wpi/nt/NetworkTableInstance.hpp"

using namespace eh;
using namespace wpi;

double PositionController::Compute(double setpoint, double measurement) {
    pidController.SetPID(pSubscriber.Get(0), iSubscriber.Get(0),
                         dSubscriber.Get(0));
    if (continuousSubscriber.Get(false)) {
        pidController.EnableContinuousInput(continuousMinimumSubscriber.Get(0),
                                            continuousMaximumSubscriber.Get(0));
    } else {
        pidController.DisableContinuousInput();
    }

    feedForward.SetKs(units::volt_t{sSubscriber.Get(0)});

    const double gLift = gLiftSubscriber.Get(0);
    const double gArm = gArmSubscriber.Get(0);
    const double gArmRatio = gArmRatioSubscriber.Get(0);
    constexpr double kGravityCompensationZeroTolerance = 1e-9;

    // Precedence rule: kgLift wins when it is configured to a nonzero value.
    // Only when kgLift is effectively zero do we fall back to arm-style
    // gravity compensation based on kgArm and kgArmRatio.
    double gravityCompensation = gLift;
    if (std::abs(gravityCompensation) <= kGravityCompensationZeroTolerance &&
        std::abs(gArm) > kGravityCompensationZeroTolerance) {
        const double armAngleRadians =
            measurement * gArmRatio * 2.0 * std::numbers::pi;
        gravityCompensation = gArm * std::cos(armAngleRadians);
    }

    return (feedForward.Calculate(
                units::meters_per_second_t{setpoint - measurement}) +
            units::volt_t{gravityCompensation} +
            units::volt_t{pidController.Calculate(measurement, setpoint)})
        .value();
}

void PositionController::Initialize(
    const wpi::nt::NetworkTableInstance& instance, const std::string& motorNum,
    const std::string& busIdStr, wpi::nt::PubSubOptions options) {
    pSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/position/kp")
                      .Subscribe(0, options);

    iSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/position/ki")
                      .Subscribe(0, options);

    dSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/position/kd")
                      .Subscribe(0, options);

    sSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/position/ks")
                      .Subscribe(0, options);

    continuousSubscriber =
        instance
            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                             "/constants/position/continuous")
            .Subscribe(false, options);

    continuousMinimumSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/constants/position/continuousMinimum")
            .Subscribe(false, options);

    continuousMaximumSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/constants/position/continuousMaximum")
            .Subscribe(false, options);

    gLiftSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/constants/position/kgLift")
            .Subscribe(0, options);

    gArmSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/constants/position/kgArm")
            .Subscribe(0, options);

    gArmRatioSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/constants/position/kgArmRatio")
            .Subscribe(0, options);
}
