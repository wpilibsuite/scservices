#include "PidConstants.h"

#include "networktables/NetworkTableInstance.h"

using namespace eh;

double PidConstants::ComputeVelocity(double setpoint, double currentPosition,
                                     double currentVelocity) {
    pidController.SetPID(pSubscriber.Get(0), iSubscriber.Get(0),
                         dSubscriber.Get(0));
    if (continuousSubscriber.Get(false)) {
        pidController.EnableContinuousInput(continuousMinimumSubscriber.Get(0),
                                            continuousMaximumSubscriber.Get(0));
    } else {
        pidController.DisableContinuousInput();
    }

    feedForward.SetKs(units::volt_t{sSubscriber.Get(0)});
    feedForward.SetKv(units::volt_t{vSubscriber.Get(0)} / 1_mps);
    feedForward.SetKa(units::volt_t{aSubscriber.Get(0)} / 1_mps_sq);

    return (feedForward.Calculate(units::meters_per_second_t{currentVelocity}) +
            units::volt_t{pidController.Calculate(currentVelocity, setpoint)})
        .value();
}

double PidConstants::ComputePosition(double setpoint, double currentPosition,
                                     double currentVelocity) {
    pidController.SetPID(pSubscriber.Get(0), iSubscriber.Get(0),
                         dSubscriber.Get(0));
    if (continuousSubscriber.Get(false)) {
        pidController.EnableContinuousInput(continuousMinimumSubscriber.Get(0),
                                            continuousMaximumSubscriber.Get(0));
    } else {
        pidController.DisableContinuousInput();
    }

    feedForward.SetKs(units::volt_t{sSubscriber.Get(0)});
    feedForward.SetKv(units::volt_t{vSubscriber.Get(0)} / 1_mps);
    feedForward.SetKa(units::volt_t{aSubscriber.Get(0)} / 1_mps_sq);

    return (feedForward.Calculate(units::meters_per_second_t{currentVelocity}) +
            units::volt_t{pidController.Calculate(currentPosition, setpoint)})
        .value();
}

void PidConstants::Initialize(const nt::NetworkTableInstance& instance,
                              const std::string& motorNum,
                              const std::string& busIdStr,
                              const std::string& pidType,
                              nt::PubSubOptions options) {
    pSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/pid/" + pidType + "/kp")
                      .Subscribe(0, options);

    iSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/pid/" + pidType + "/ki")
                      .Subscribe(0, options);

    dSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/pid/" + pidType + "/kd")
                      .Subscribe(0, options);

    aSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/pid/" + pidType + "/ka")
                      .Subscribe(0, options);

    vSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/pid/" + pidType + "/kv")
                      .Subscribe(0, options);

    sSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/pid/" + pidType + "/ks")
                      .Subscribe(0, options);

    continuousSubscriber =
        instance
            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                             "/pid/" + pidType + "/continuous")
            .Subscribe(false, options);

    continuousMinimumSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/pid/" + pidType + "/continuousMinimum")
            .Subscribe(false, options);

    continuousMaximumSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNum +
                            "/pid/" + pidType + "/continousMaximum")
            .Subscribe(false, options);
}
