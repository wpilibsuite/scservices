#include "version.h"

#define STR_HELPER(x) #x
#define STR_VER(x) STR_HELPER(x)

// Leave these as globals. We want them accessable in the debugger.
const char* MRC_GitHash =
#ifndef MRC_BUILD_GIT_HASH
    "Unknown";
#else
    STR_VER(MRC_BUILD_GIT_HASH);
#endif

const char* MRC_BuildTimestamp =
#ifndef MRC_BUILD_TIMESTAMP
    "Unknown";
#else
    STR_VER(MRC_BUILD_TIMESTAMP);
#endif

extern "C" {

const char* MRC_GetGitHash(void) {
    return MRC_GitHash;
}

const char* MRC_GetBuildTimestamp(void) {
    return MRC_BuildTimestamp;
}
}
