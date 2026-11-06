# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# The per-file include list exceeds the 32 KiB CreateProcess limit on Windows;
# make Ninja pass compile flags through response files instead.
if(CMAKE_HOST_WIN32)
    set(CMAKE_NINJA_FORCE_RESPONSE_FILE ON CACHE INTERNAL "")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Only build the components that main depends on (directly or transitively).
idf_build_set_property(MINIMAL_BUILD ON)
project(esp32-p4-lcd-4-3-brookesia)

if(TARGET __idf_espressif__mcp-c-sdk AND CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(__idf_espressif__mcp-c-sdk PRIVATE
        -Wno-error=stringop-truncation
    )
endif()

if(TARGET __idf_espressif__esp-boost AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(__idf_espressif__esp-boost PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=deprecated-variadic-comma-omission>
    )
endif()
