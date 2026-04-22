#pragma once

#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>
#include <wpi/units/acceleration.hpp>

namespace eh {
  inline static constexpr auto Ks = 0_V;
  inline static constexpr auto Kv = 0_V / 1_mps;
  inline static constexpr auto Ka = 0_V / 1_mps_sq;

  inline static constexpr auto Period = 12_ms;
}
