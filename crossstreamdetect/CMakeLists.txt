project(CanCrossStreamDetectorDaemon)

add_executable(CanCrossStreamDetectorDaemon main.cpp)
target_compile_features(CanCrossStreamDetectorDaemon PRIVATE cxx_std_20)
wpilib_target_warnings(CanCrossStreamDetectorDaemon)
target_link_libraries(CanCrossStreamDetectorDaemon PRIVATE Common ntcore wpinet wpiutil)

if (MRC_BUILD)
target_compile_definitions(CanCrossStreamDetectorDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
