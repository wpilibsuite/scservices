project(ExpansionHubDaemon)

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--allow-shlib-undefined")

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
target_link_libraries(ExpansionHubDaemon PRIVATE Common wpimath ntcore wpinet wpiutil)
target_link_libraries(ExpansionHubDaemon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libsystemd.so.0.37.0)
target_include_directories(ExpansionHubDaemon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/rhsp/include)

if (MRC_BUILD)
target_compile_definitions(ExpansionHubDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
