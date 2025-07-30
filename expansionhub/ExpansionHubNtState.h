#pragma once

#include <array>
#include "MotorNtState.h"
#include "ServoNtState.h"

#define NUM_MOTORS_PER_HUB 4
#define NUM_SERVOS_PER_HUB 6

namespace eh {

struct ExpansionHubNtState {
    std::array<MotorNtState, NUM_MOTORS_PER_HUB> motors;
    std::array<ServoNtState, NUM_SERVOS_PER_HUB> servos;

    double lastBattery{0};

    nt::DoublePublisher batteryVoltagePublisher;

    nt::BooleanPublisher isConnectedPublisher;

    nt::IntegerPublisher numNacksPublisher;
    nt::IntegerPublisher numCrcFailuresPublisher;
    nt::IntegerPublisher numMissedSendLoopsPublisher;

    nt::IntegerPublisher transactionTimePublisher;

    uint64_t numNacks{0};
    uint64_t numCrcFailures{0};
    uint64_t numMissedSendLoops{0};

    void Initialize(const nt::NetworkTableInstance& instance, int deviceNum);
};

}  // namespace eh
