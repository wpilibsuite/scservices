project(KitCanDaemon)

add_executable(KitCanDaemon main.cpp)
target_compile_features(KitCanDaemon PRIVATE cxx_std_20)
wpilib_target_warnings(KitCanDaemon)
target_link_libraries(KitCanDaemon PRIVATE Common ntcore wpinet wpiutil)

if (MRC_BUILD)
target_compile_definitions(KitCanDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
