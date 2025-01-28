project(PowerDistributionDaemon)

add_executable(PowerDistributionDaemon main.cpp)
target_compile_features(PowerDistributionDaemon PRIVATE cxx_std_20)
wpilib_target_warnings(PowerDistributionDaemon)
target_link_libraries(PowerDistributionDaemon PRIVATE Common ntcore wpinet wpiutil)

if (MRC_BUILD)
target_compile_definitions(PowerDistributionDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
