#pragma once

#include <string>

#include "ControllerShared.h"

#include <wpi/units/length.hpp>
#include <wpi/units/voltage.hpp>

#include "wpi/math/controller/PIDController.hpp"
#include "wpi/math/controller/SimpleMotorFeedforward.hpp"

#include "wpi/nt/DoubleTopic.hpp"

namespace eh {

struct VelocityController {
    wpi::nt::DoubleSubscriber pSubscriber;
    wpi::nt::DoubleSubscriber iSubscriber;
    wpi::nt::DoubleSubscriber dSubscriber;
    wpi::nt::DoubleSubscriber sSubscriber;
    wpi::nt::DoubleSubscriber vSubscriber;
    wpi::nt::DoubleSubscriber aSubscriber;

    wpi::math::PIDController pidController{0, 0, 0, eh::Period};
    wpi::math::SimpleMotorFeedforward<wpi::units::meter> feedForward{eh::Ks, eh::Kv, eh::Ka, Period};

    void Initialize(const wpi::nt::NetworkTableInstance& instance,
                    const std::string& motorNum, const std::string& busIdStr,
                    wpi::nt::PubSubOptions options);

    double Compute(double setpoint, double measurement);
};

}  // namespace eh
