add_library(systemd SHARED IMPORTED GLOBAL)

set_target_properties(systemd PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libsystemd.so.0.37.0"
    IMPORTED_SONAME "libsystemd.so.0"
)

target_include_directories(systemd INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_options(systemd INTERFACE
    "LINKER:--unresolved-symbols=ignore-in-shared-libs"
)
