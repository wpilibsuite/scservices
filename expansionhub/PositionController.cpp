#include "PositionController.h"

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

    return (feedForward.Calculate(
                units::meters_per_second_t{measurement - setpoint}) +
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
}
