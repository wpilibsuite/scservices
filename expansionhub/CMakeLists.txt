project(ExpansionHubDaemon)

add_executable(ExpansionHubDaemon
    main.cpp
    PidConstants.cpp
    SerialPort.cpp
    EnabledState.cpp
    MotorNtState.cpp
    ServoNtState.cpp
    ExpansionHubNtState.cpp
    ExpansionHubSerial.cpp
    SystemDUsbMonitor.cpp
)
target_compile_features(ExpansionHubDaemon PRIVATE cxx_std_20)
target_compile_features(ExpansionHubDaemon PRIVATE c_std_17)
wpilib_target_warnings(ExpansionHubDaemon)
target_link_libraries(ExpansionHubDaemon PRIVATE Common wpimath ntcore wpinet wpiutil systemd)

if (MRC_BUILD)
target_compile_definitions(ExpansionHubDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
