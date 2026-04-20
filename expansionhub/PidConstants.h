#pragma once

#include <string>

#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>
#include <wpi/units/acceleration.hpp>

#include "wpi/math/controller/PIDController.hpp"
#include "wpi/math/controller/SimpleMotorFeedforward.hpp"

#include "wpi/nt/BooleanTopic.hpp"
#include "wpi/nt/DoubleTopic.hpp"

namespace eh {

struct PidConstants {
    wpi::nt::DoubleSubscriber pSubscriber;
    wpi::nt::DoubleSubscriber iSubscriber;
    wpi::nt::DoubleSubscriber dSubscriber;
    wpi::nt::DoubleSubscriber sSubscriber;
    wpi::nt::DoubleSubscriber vSubscriber;
    wpi::nt::DoubleSubscriber aSubscriber;

    wpi::nt::BooleanSubscriber continuousSubscriber;
    wpi::nt::DoubleSubscriber continuousMinimumSubscriber;
    wpi::nt::DoubleSubscriber continuousMaximumSubscriber;

    inline static constexpr auto Ks = 0_V;
    inline static constexpr auto Kv = 0_V / 1_mps;
    inline static constexpr auto Ka = 0_V / 1_mps_sq;

    inline static constexpr auto Period = 12_ms;

    wpi::math::PIDController pidController{0, 0, 0, Period};
    // Yes this says meters but its unitless.
    wpi::math::SimpleMotorFeedforward<wpi::units::meter> feedForward{Ks, Kv, Ka, Period};

    void Initialize(const wpi::nt::NetworkTableInstance& instance,
                    const std::string& motorNum, const std::string& busIdStr,
                    const std::string& pidType, wpi::nt::PubSubOptions options);

    double Compute(double setpoint, double measurement);
};

}  // namespace eh
