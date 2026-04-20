project(MotioncoreOdometryDaemon)

add_executable(MotioncoreOdometryDaemon src/main.cpp src/proto/Motioncore.npb.cpp src/MotioncoreProto.cpp)
target_compile_features(MotioncoreOdometryDaemon PRIVATE cxx_std_20)
wpilib_target_warnings(MotioncoreOdometryDaemon)
target_link_libraries(MotioncoreOdometryDaemon PRIVATE Common wpimath ntcore wpinet wpiutil)

if (MRC_BUILD)
target_compile_definitions(MotioncoreOdometryDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
