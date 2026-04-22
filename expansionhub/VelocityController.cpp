#include "VelocityController.h"

#include "wpi/nt/NetworkTableInstance.hpp"

using namespace eh;
using namespace wpi;

double VelocityController::Compute(double setpoint, double measurement) {
    pidController.SetPID(pSubscriber.Get(0), iSubscriber.Get(0),
                         dSubscriber.Get(0));

    feedForward.SetKs(units::volt_t{sSubscriber.Get(0)});
    feedForward.SetKv(units::volt_t{vSubscriber.Get(0)} / 1_mps);
    feedForward.SetKa(units::volt_t{aSubscriber.Get(0)} / 1_mps_sq);

    return (feedForward.Calculate(units::meters_per_second_t{setpoint}) +
            units::volt_t{pidController.Calculate(measurement, setpoint)})
        .value();
}

void VelocityController::Initialize(const wpi::nt::NetworkTableInstance& instance,
                              const std::string& motorNum,
                              const std::string& busIdStr,
                              wpi::nt::PubSubOptions options) {
    pSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/velocity/kp")
                      .Subscribe(0, options);

    iSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/velocity/ki")
                      .Subscribe(0, options);

    dSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/velocity/kd")
                      .Subscribe(0, options);

    aSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/velocity/ka")
                      .Subscribe(0, options);

    vSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/velocity/kv")
                      .Subscribe(0, options);

    sSubscriber = instance
                      .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                      motorNum + "/constants/velocity/ks")
                      .Subscribe(0, options);
}
