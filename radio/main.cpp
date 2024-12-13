#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include "version.h"

#include <wpinet/EventLoopRunner.h>
#include <wpinet/uv/Timer.h>
#include <wpinet/uv/Tcp.h>
#include "wpinet/HttpUtil.h"

#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include "wpi/StringExtras.h"

struct DataStorage {
    nt::StringSubscriber teamSubscriber;

    std::shared_ptr<wpi::uv::Tcp> tcpConn;
    std::optional<uint16_t> oldTeamNumber;
};

static bool startUvLoop(wpi::uv::Loop& loop, DataStorage& instData);

int main() {
    printf("Starting RadioDaemon\n");
    printf("\tBuild Hash: %s\n", MRC_GetGitHash());
    printf("\tBuild Timestamp: %s\n", MRC_GetBuildTimestamp());

#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
#endif

    auto ntInst = nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient4("RadioDaemon");

    DataStorage instData;

    instData.teamSubscriber = ntInst.GetStringTopic("/sys/team").Subscribe("");

    wpi::EventLoopRunner loopRunner;

    bool success = false;
    loopRunner.ExecSync([&success, &instData](wpi::uv::Loop& loop) {
        success = startUvLoop(loop, instData);
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
    loopRunner.Stop();
    ntInst.StopClient();
    nt::NetworkTableInstance::Destroy(ntInst);

    return 0;
}

static std::optional<uint16_t> checkTeamNumber(DataStorage& instData) {
    auto teamStr = instData.teamSubscriber.Get();
    auto maybeTeam = wpi::parse_integer<uint16_t>(teamStr, 10);
    if (!maybeTeam.has_value()) {
        printf("failed to parse team number\n");
        return {};
    }

    auto team = *maybeTeam;

    if (team > 25599) {
        printf("Team number too large\n");
        return {};
    }

    return team;
}

static void handleNewConnection(wpi::uv::Loop& loop, DataStorage& instData) {
    auto tcp = wpi::uv::Tcp::Create(loop, 0);

    tcp->error.connect
}

static void handleTeamUpdate(wpi::uv::Loop& loop, DataStorage& instData) {}

static bool startUvLoop(wpi::uv::Loop& loop, DataStorage& instData) {
    auto timer = wpi::uv::Timer::Create(loop);
    if (!timer) {
        return false;
    }

    timer->timeout.connect([&loop, &instData] {
        auto maybeTeam = checkTeamNumber(instData);
        if (!maybeTeam.has_value()) {
            instData.oldTeamNumber = maybeTeam;
            return;
        }

        if (maybeTeam != instData.oldTeamNumber) {
            // New team number.
            instData.oldTeamNumber = maybeTeam;
            // Restart TCP if it exists.

            if (instData.tcpConn) {
                instData.tcpConn->Shutdown();
                instData.tcpConn->Close();
                instData.tcpConn = nullptr;
            }

            handleNewConnection(loop, instData);
            return;
        }

        handleTeamUpdate(loop, instData);

        //    fmt::format("10.{}.{}.1", static_cast<int>(team / 100),
        //                static_cast<int>(team % 100));

        //     auto tcp = wpi::uv::Tcp::Create(loop, 0);
        //     if (!tcp) {
        //         printf("Failed to allocate tcp\n");
        //         return;
        //     }

        //     tcp->Reuse
    });

    timer->Start(wpi::uv::Timer::Time{100}, wpi::uv::Timer::Time{3000});

    return true;
}
