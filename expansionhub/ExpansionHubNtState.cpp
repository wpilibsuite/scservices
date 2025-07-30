#include "ExpansionHubNtState.h"

#include "networktables/NetworkTableInstance.h"

using namespace eh;

void ExpansionHubNtState::Initialize(const nt::NetworkTableInstance& instance,
                                    int deviceNum) {
    if (isConnectedPublisher) {
        return;
    }

    nt::PubSubOptions options;
    options.sendAll = true;
    options.keepDuplicates = true;
    options.periodic = 0.005;

    auto busIdStr = std::to_string(deviceNum);

    for (int i = 0; i < static_cast<int>(motors.size()); i++) {
        motors[i].Initialize(instance, i, busIdStr, options);
    }

    for (int i = 0; i < static_cast<int>(servos.size()); i++) {
        servos[i].Initialize(instance, i, busIdStr, options);
    }

    transactionTimePublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/transactionTime")
            .Publish(options);

    batteryVoltagePublisher =
        instance.GetDoubleTopic("/rhsp/" + busIdStr + "/battery")
            .Publish(options);
    isConnectedPublisher =
        instance.GetBooleanTopic("/rhsp/" + busIdStr + "/connected")
            .Publish(options);

    numCrcFailuresPublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/numCrcFailures")
            .Publish(options);

    numMissedSendLoopsPublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/numMissedSendLoops")
            .Publish(options);

    numNacksPublisher =
        instance.GetIntegerTopic("/rhsp/" + busIdStr + "/numNacks")
            .Publish(options);
}
