project(Common)

set (SOURCES
    version.cpp
)

add_library(Common STATIC ${SOURCES})
target_compile_features(Common PUBLIC cxx_std_20)
wpilib_target_warnings(Common)
target_include_directories(Common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(Common PRIVATE MRC_BUILD_GIT_HASH=${BUILD_CURRENT_GIT_HASH})
target_compile_definitions(Common PRIVATE MRC_BUILD_TIMESTAMP=${BUILD_CURRENT_TIMESTAMP})
