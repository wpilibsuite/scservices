# Disable in-source builds to prevent source tree corruption.
if(" ${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL " ${CMAKE_CURRENT_BINARY_DIR}")
    message(
        FATAL_ERROR
        "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
"
    )
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.21)

project(scservices)

# Set default build type to release with debug info (i.e. release mode optimizations
# are performed, but debug info still exists).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build with shared libs (needed for JNI)" OFF)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(WITH_NTCORE "Include ntcore" ON)
option(WITH_JAVA "Include java and JNI in the build" OFF)
option(WITH_CSCORE "Build cscore (needs OpenCV)" OFF)
option(WITH_WPIMATH "Build wpimath" OFF)
option(WITH_WPILIB "Build hal, wpilibc/j, and myRobot (needs OpenCV)" OFF)
option(WITH_OLD_COMMANDS "Build old commands" OFF)
option(WITH_EXAMPLES "Build examples" OFF)
option(WITH_TESTS "Build unit tests (requires internet connection)" OFF)
option(WITH_GUI "Build GUI items" OFF)
option(WITH_SIMULATION_MODULES "Build simulation modules" OFF)
option(WITH_PROTOBUF "Build protobuf support" OFF)

include(${PROJECT_SOURCE_DIR}/cmake/GitCommands.cmake)
get_git_current_hash(${PROJECT_SOURCE_DIR} BUILD_CURRENT_GIT_HASH)

if(NOT BUILD_CURRENT_GIT_HASH)
    message("Failed to get git hash. Binary will not contain git hash")
    set(GIT_CURRENT_HASH "Unknown")
    set(BUILD_CURRENT_GIT_HASH "Unknown")
endif()

string(TIMESTAMP BUILD_CURRENT_TIMESTAMP UTC)

if (MSVC)
    if ("${CMAKE_C_COMPILER_VERSION}" VERSION_GREATER_EQUAL "19.20")
        include(${PROJECT_SOURCE_DIR}/cmake/SourceLink.cmake)
        file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/source_link.json" SOURCE_LINK_JSON)
        file(TO_NATIVE_PATH "${PROJECT_SOURCE_DIR}/cmake/source_link.json.in" SOURCE_LINK_JSON_INPUT)
        source_link(${PROJECT_SOURCE_DIR} ${SOURCE_LINK_JSON} ${SOURCE_LINK_JSON_INPUT})
        set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /INCREMENTAL:NO")
        set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /INCREMENTAL:NO")
        set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /INCREMENTAL:NO")
        set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /INCREMENTAL:NO")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:PGOREPRO /SOURCELINK:\"${SOURCE_LINK_JSON}\"")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /FORCE:PGOREPRO /SOURCELINK:\"${SOURCE_LINK_JSON}\"")
    else()
        message(WARNING "Disabling SourceLink due to old version of MSVC. Please update to VS2019 or greater!")
    endif()
endif()

add_subdirectory(allwpilib EXCLUDE_FROM_ALL)

set(BUILD_SHARED_LIBS OFF)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS FALSE)

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1 # or a later release
)

FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)

add_subdirectory(common)

add_subdirectory(radio)

if (MRC_BUILD)
add_subdirectory(powerdistribution)
add_subdirectory(crossstreamdetect)
endif()

if (MRC_BUILD)
install(
    TARGETS RadioDaemon PowerDistributionDaemon CanCrossStreamDetectorDaemon
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

else()
install(
    TARGETS RadioDaemon
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
endif()
