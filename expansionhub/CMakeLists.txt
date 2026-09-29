project(ExpansionHubDaemon)

add_executable(ExpansionHubDaemon
    main.cpp
    PositionController.cpp
    VelocityController.cpp
    SerialPort.cpp
    EnabledState.cpp
    MotorNtState.cpp
    ServoNtState.cpp
    ExpansionHubNtState.cpp
    ExpansionHubSerial.cpp
    SystemDUsbMonitor.cpp
)

find_library(SYSTEMD REQUIRED NAMES systemd libsystemd.so.0)
message(STATUS "Found libsystemd: ${SYSTEMD}")
check_include_file("systemd/sd-device.h" HAVE_SYSTEMD_SD_DEVICE_H)
if (NOT HAVE_SYSTEMD_SD_DEVICE_H)
    message(FATAL_ERROR "systemd/sd-device.h not found")
endif()

target_compile_features(ExpansionHubDaemon PRIVATE cxx_std_20)
target_compile_features(ExpansionHubDaemon PRIVATE c_std_17)
wpilib_target_warnings(ExpansionHubDaemon)
target_link_libraries(ExpansionHubDaemon PRIVATE Common wpimath ntcore wpinet wpiutil)
target_link_libraries(ExpansionHubDaemon PRIVATE ${SYSTEMD})
target_include_directories(ExpansionHubDaemon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/rhsp/include)

if (MRC_BUILD)
target_compile_definitions(ExpansionHubDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
