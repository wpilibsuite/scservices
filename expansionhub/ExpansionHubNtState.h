#pragma once

#include <array>
#include "MotorNtState.h"
#include "ServoNtState.h"

#define NUM_MOTORS_PER_HUB 4
#define NUM_SERVOS_PER_HUB 6
#define NUM_ANALOG_INPUTS_PER_HUB 4

namespace eh {

struct ExpansionHubNtState {
    std::array<MotorNtState, NUM_MOTORS_PER_HUB> motors;
    std::array<ServoNtState, NUM_SERVOS_PER_HUB> servos;
    std::array<wpi::nt::IntegerPublisher, NUM_ANALOG_INPUTS_PER_HUB> analogPublishers;

    double lastBattery{0};

    wpi::nt::DoublePublisher batteryVoltagePublisher;

    wpi::nt::BooleanPublisher isConnectedPublisher;

    wpi::nt::IntegerPublisher numNacksPublisher;
    wpi::nt::IntegerPublisher numCrcFailuresPublisher;
    wpi::nt::IntegerPublisher numMissedSendLoopsPublisher;

    wpi::nt::IntegerPublisher transactionTimePublisher;

    uint64_t numNacks{0};
    uint64_t numCrcFailures{0};
    uint64_t numMissedSendLoops{0};

    void Initialize(const wpi::nt::NetworkTableInstance& instance, int deviceNum);
};

}  // namespace eh
