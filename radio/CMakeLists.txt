project(RadioDaemon)

add_executable(RadioDaemon main.cpp)
target_compile_features(RadioDaemon PRIVATE cxx_std_20)
wpilib_target_warnings(RadioDaemon)
target_link_libraries(RadioDaemon PRIVATE Common ntcore wpinet wpiutil)

if (MRC_BUILD)
target_compile_definitions(RadioDaemon PRIVATE MRC_DAEMON_BUILD)
endif()
