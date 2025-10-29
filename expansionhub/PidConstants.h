#pragma once

#include <string>

#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/acceleration.h>

#include "frc/controller/PIDController.h"
#include "frc/controller/SimpleMotorFeedforward.h"

#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"

namespace eh {

struct PidConstants {
    nt::DoubleSubscriber pSubscriber;
    nt::DoubleSubscriber iSubscriber;
    nt::DoubleSubscriber dSubscriber;
    nt::DoubleSubscriber sSubscriber;
    nt::DoubleSubscriber vSubscriber;
    nt::DoubleSubscriber aSubscriber;

    nt::BooleanSubscriber continuousSubscriber;
    nt::DoubleSubscriber continuousMinimumSubscriber;
    nt::DoubleSubscriber continuousMaximumSubscriber;

    inline static constexpr auto Ks = 0_V;
    inline static constexpr auto Kv = 0_V / 1_mps;
    inline static constexpr auto Ka = 0_V / 1_mps_sq;

    inline static constexpr auto Period = 12_ms;

    frc::PIDController pidController{0, 0, 0, Period};
    // Yes this says meters but its unitless.
    frc::SimpleMotorFeedforward<units::meter> feedForward{Ks, Kv, Ka};

    void Initialize(const nt::NetworkTableInstance& instance,
                    const std::string& motorNum, const std::string& busIdStr,
                    const std::string& pidType, nt::PubSubOptions options);

    double Compute(double setpoint, double measurement);
};

}  // namespace eh
