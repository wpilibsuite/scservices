#include "ServoNtState.h"

#include "networktables/NetworkTableInstance.h"

using namespace eh;

void ServoNtState::Initialize(const nt::NetworkTableInstance& instance,
                            int servoNum, const std::string& busIdStr,
                            nt::PubSubOptions options) {
    auto servoNumStr = std::to_string(servoNum);
    enabledSubscriber = instance
                            .GetBooleanTopic("/rhsp/" + busIdStr + "/servo" +
                                             servoNumStr + "/enabled")
                            .Subscribe(false, options);
    framePeriodSubscriber =
        instance
            .GetIntegerTopic("/rhsp/" + busIdStr + "/servo" + servoNumStr +
                             "/framePeriod")
            .Subscribe(20000, options);
    pulseWidthSubscriber = instance
                               .GetIntegerTopic("/rhsp/" + busIdStr + "/servo" +
                                                servoNumStr + "/pulseWidth")
                               .Subscribe(1500, options);
}
