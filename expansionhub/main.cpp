#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include "version.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <filesystem>

#include <wpinet/EventLoopRunner.h>
#include <wpinet/uv/Poll.h>
#include <wpinet/uv/FsEvent.h>
#include <wpinet/uv/Timer.h>

#include "networktables/NetworkTableInstance.h"
#include "networktables/RawTopic.h"

#include "networktables/IntegerTopic.h"
#include "networktables/DoubleTopic.h"

#include "systemd/sd-device.h"

#include "SerialPort.h"

#include <deque>

#include "networktables/BooleanTopic.h"

#include <wpi/timestamp.h>

#include "ReceiveStateMachine.h"
#include "MessageNumbers.h"

#include "frc/controller/PIDController.h"
#include "frc/controller/SimpleMotorFeedforward.h"

#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/acceleration.h>

#include "ExpansionHubNtState.h"
#include "ExpansionHubSerial.h"
#include "EnabledState.h"
#include "SystemDUsbMonitor.h"

struct ExpansionHubState {
    uint64_t lastLoop = wpi::Now();

    int socketHandle{-1};

    eh::ExpansionHubNtState ntStore;

    const nt::NetworkTableInstance* ntInstance;

    unsigned busId{0};

    std::unique_ptr<eh::ExpansionHubSerial> currentHub;

    ~ExpansionHubState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void OnUpdate(bool canEnable);

    void SendInitial();

    void SendCommands(bool canEnable, bool deviceReset);

    bool StartUvLoop(unsigned bus, const nt::NetworkTableInstance& ntInst,
                     wpi::uv::Loop& loop);

    void OnDeviceAdded(std::unique_ptr<eh::ExpansionHubSerial> hub);

    void OnDeviceRemoved(std::string_view port);
};

void ExpansionHubState::SendInitial() {
    // Make sure keep alive is the first thing sent, its needed for recovery
    currentHub->SendKeepAlive();
    currentHub->GetModuleStatus();

    // Do encoder resets next
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        auto reset = ntStore.motors[i].resetEncoderSubscriber.GetAtomic(false);
        if (reset.time != ntStore.motors[i].lastResetTime) {
            ntStore.motors[i].lastResetTime = reset.time;
            ntStore.motors[i].doReset = true;
        }

        if (ntStore.motors[i].doReset) {
            currentHub->SendEncoderResetRequest(i);
        }
    }

    // Do requests next
    currentHub->SendBatteryRequest();
    currentHub->SendBulkInput();

    // Then the servos
    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        currentHub->SendServoPulseWidth(
            i, ntStore.servos[i].pulseWidthSubscriber.Get(1500));
    }

    currentHub->Flush();
}

void ExpansionHubState::SendCommands(bool canEnable, bool deviceReset) {
    // We've unrolled these so we can control updates together to make
    // sure they happen as close as possible.

    if (deviceReset) {
        for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
            ntStore.motors[i].enabledSubscriber.ForceReset();
            ntStore.motors[i].floatOn0Subscriber.ForceReset();
        }

        for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
            ntStore.servos[i].enabledSubscriber.ForceReset();
            ntStore.servos[i].framePeriodSubscriber.ForceReset();
        }
    }

    // First the 4 motors.
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        currentHub->SendMotorConstantPower(
            i, ntStore.motors[i].ComputeMotorPower(ntStore.lastBattery));
    }

    // Then get the motor currents
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        currentHub->SendMotorCurrentRequest(i);
    }

    // Then all the things that are not expect to change, but we want to keep
    // sending in case of reset.
    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        auto sendFloatOn0 = ntStore.motors[i].floatOn0Subscriber.Get();
        if (sendFloatOn0.has_value()) {
            currentHub->SendMotorMode(i, *sendFloatOn0);
        }
    }

    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        auto sendEnable =
            ntStore.motors[i].enabledSubscriber.GetWithCanEnable(canEnable);
        if (sendEnable.has_value()) {
            currentHub->SendMotorEnable(i, *sendEnable);
        }
    }

    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        auto servoConfig = ntStore.servos[i].framePeriodSubscriber.Get();
        if (servoConfig.has_value()) {
            currentHub->SendServoConfiguration(i, *servoConfig);
        }
    }

    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        auto sendEnable =
            ntStore.servos[i].enabledSubscriber.GetWithCanEnable(canEnable);
        if (sendEnable.has_value()) {
            currentHub->SendServoEnable(i, *sendEnable);
        }
    }

    currentHub->Flush();
}

void ExpansionHubState::OnDeviceAdded(std::unique_ptr<eh::ExpansionHubSerial> hub) {
    currentHub = std::move(hub);

    ntStore.Initialize(*ntInstance, busId);

    ntStore.isConnectedPublisher.Set(true);
    printf("Device added\n");
    ntStore.numNacks = 0;
    ntStore.numNacksPublisher.Set(ntStore.numNacks);
    ntStore.numCrcFailures = 0;
    ntStore.numCrcFailuresPublisher.Set(ntStore.numCrcFailures);
    ntStore.numMissedSendLoops = 0;
    ntStore.numMissedSendLoopsPublisher.Set(ntStore.numMissedSendLoops);

    for (int i = 0; i < NUM_MOTORS_PER_HUB; i++) {
        ntStore.motors[i].enabledSubscriber.ForceReset();
        ntStore.motors[i].floatOn0Subscriber.ForceReset();
    }

    for (int i = 0; i < NUM_SERVOS_PER_HUB; i++) {
        ntStore.servos[i].enabledSubscriber.ForceReset();
        ntStore.servos[i].framePeriodSubscriber.ForceReset();
    }

    currentHub->SetCallbacks(
        [this](bool canEnable, bool deviceReset) {
            SendCommands(canEnable, deviceReset);
        },
        &ntStore);

    // TODO we only want the timer running if we have a device.
}

void ExpansionHubState::OnDeviceRemoved(std::string_view path) {
    if (currentHub && path == currentHub->SerialPath()) {
        ntStore.isConnectedPublisher.Set(false);
        currentHub.reset();
    }
}

void ExpansionHubState::OnUpdate(bool canEnable) {
    if (!currentHub) {
        return;
    }

    if (!currentHub->HasFinishedSynchronous()) {
        currentHub->RunSynchronousSteps();
        return;
    }

    auto now = wpi::Now();
    auto delta = now - lastLoop;

    bool allowSend = currentHub->AllowSend();

    if (!allowSend && delta < 1000000) {
        printf("Skipping due to outstanding\n");
        ntStore.numMissedSendLoops++;
        ntStore.numMissedSendLoopsPublisher.Set(ntStore.numMissedSendLoops);
        return;
    } else if (!allowSend && delta >= 1000000) {
        printf("1 second timeout. Attempting to recover\n");
        currentHub->Recover();
    }

    lastLoop = now;

    if (delta > 23000) {
        printf("Delta time %lu\n", delta);
    }

    currentHub->StartTransaction(canEnable);

    SendInitial();
}

static void OnDeviceRemoved(
    std::array<ExpansionHubState, NUM_USB_BUSES>& states,
    const std::string& devPath) {
    std::string_view path = devPath;

    for (auto&& i : states) {
        i.OnDeviceRemoved(path);
    }
}

static void OnDeviceAdded(wpi::uv::Loop& loop,
                          std::array<ExpansionHubState, NUM_USB_BUSES>& states,
                          int busNum, const std::string& devPath) {
    if (states[busNum].currentHub != nullptr) {
        printf("Received duplicate bus, likely race condition\n");
        return;
    }

    int ret = eh::OpenRhspSerialPort(devPath.c_str());
    if (ret < 0) {
        printf("OpenRhspSerialPort failed %d\n", ret);
        return;
    }

    std::unique_ptr<eh::ExpansionHubSerial> sl = std::make_unique<eh::ExpansionHubSerial>();
    bool isInit = sl->Initialize(loop, ret, devPath);
    printf("initialized %d\n", isInit ? 1 : 0);

    sl->RunSynchronousSteps();

    states[busNum].OnDeviceAdded(std::move(sl));
}

bool ExpansionHubState::StartUvLoop(unsigned bus,
                                    const nt::NetworkTableInstance& ntInst,
                                    wpi::uv::Loop& loop) {
    if (bus >= NUM_USB_BUSES) {
        return false;
    }

    busId = bus;
    ntInstance = &ntInst;

    return true;
}

int main() {
    printf("Starting ExpansionHubDaemon\n");
    printf("\tBuild Hash: %s\n", MRC_GetGitHash());
    printf("\tBuild Timestamp: %s\n", MRC_GetBuildTimestamp());

#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
#endif

    eh::EnabledState enabledState;

    if (!enabledState.Initialize()) {
        printf("Failed to open control data.\n");
        return -1;
    }

    std::array<ExpansionHubState, NUM_USB_BUSES> states;
    eh::SystemDUsbMonitor usbMonitor{
        [&states](wpi::uv::Loop& loop, int busNum, const std::string& devPath) {
            OnDeviceAdded(loop, states, busNum, devPath);
        },
        [&states](const std::string& devPath) {
            OnDeviceRemoved(states, devPath);
        }};

    auto ntInst = nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient("ExpansionHubDaemon");

    wpi::EventLoopRunner loopRunner;

    struct LoopStorage {
        std::array<ExpansionHubState, NUM_USB_BUSES>* hubStates;
        wpi::uv::Loop* loop;
    } loopStorage{
        .hubStates = &states,
        .loop = nullptr,
    };

    bool success = false;
    loopRunner.ExecSync([&success, &states, &loopStorage, &ntInst,
                         &usbMonitor, &enabledState](wpi::uv::Loop& loop) {
        loopStorage.loop = &loop;
        for (size_t i = 0; i < states.size(); i++) {
            success = states[i].StartUvLoop(i, ntInst, loop);
            if (!success) {
                return;
            }
        }

        auto usbMonResult = usbMonitor.Initialize(&loop);
        if (!usbMonResult) {
            printf("SystemDUsbMonitor::Initialize failed\n");
            success = false;
            return;
        }

        auto poll = wpi::uv::Poll::Create(loop, usbMonitor.GetFd());
        if (!poll) {
            printf("Poll create failed\n");
            success = false;
            return;
        }

        poll->pollEvent.connect(
            [&usbMonitor](int flags) { usbMonitor.HandleEvent(); });

        poll->Start(UV_READABLE);

        auto sendTimer = wpi::uv::Timer::Create(loop);

        sendTimer->timeout.connect([&states, &enabledState]() {
            bool system_watchdog = enabledState.IsEnabled();

            for (auto&& dev : states) {
                dev.OnUpdate(system_watchdog);
            }
        });

        units::millisecond_t millis = eh::PidConstants::Period;
        uint64_t rawMillis = static_cast<uint64_t>(millis.value());

        sendTimer->Start(wpi::uv::Timer::Time{rawMillis},
                         wpi::uv::Timer::Time{rawMillis});

        // Enumerate everything for initial checking

        usbMonitor.DoInitialCheck();
    });

    if (!success) {
        loopRunner.Stop();
        return -1;
    }

    {
#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
        int sig = 0;
        sigwait(&signal_set, &sig);
#else
        (void)getchar();
#endif
    }
    ntInst.StopClient();
    nt::NetworkTableInstance::Destroy(ntInst);

    loopRunner.Stop();

    return 0;
}
