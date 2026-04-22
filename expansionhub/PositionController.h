#pragma once

#include <string>

#include "ControllerShared.h"

#include <wpi/units/length.hpp>
#include <wpi/units/voltage.hpp>

#include "wpi/math/controller/PIDController.hpp"
#include "wpi/math/controller/SimpleMotorFeedforward.hpp"

#include "wpi/nt/BooleanTopic.hpp"
#include "wpi/nt/DoubleTopic.hpp"

namespace eh {

struct PositionController {
    wpi::nt::DoubleSubscriber pSubscriber;
    wpi::nt::DoubleSubscriber iSubscriber;
    wpi::nt::DoubleSubscriber dSubscriber;
    wpi::nt::DoubleSubscriber sSubscriber;

    wpi::nt::BooleanSubscriber continuousSubscriber;
    wpi::nt::DoubleSubscriber continuousMinimumSubscriber;
    wpi::nt::DoubleSubscriber continuousMaximumSubscriber;

    wpi::math::PIDController pidController{0, 0, 0, eh::Period};
    wpi::math::SimpleMotorFeedforward<wpi::units::meter> feedForward{eh::Ks, 0_V / 1_mps, 0_V / 1_mps_sq, eh::Period};

    void Initialize(const wpi::nt::NetworkTableInstance& instance,
                    const std::string& motorNum, const std::string& busIdStr,
                    wpi::nt::PubSubOptions options);

    double Compute(double setpoint, double measurement);
};

}  // namespace eh
