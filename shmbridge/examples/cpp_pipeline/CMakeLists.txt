cmake_minimum_required(VERSION 3.16)
project(cpp_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Locate shmbridge ──────────────────────────────────────────────────────────
# Search order:
#   1. Parent build tree (monorepo: shmbridge built as part of the repo)
#   2. Installed system / user package
if(NOT TARGET shmbridge::shmbridge)
    find_package(shmbridge QUIET)
endif()

if(NOT TARGET shmbridge::shmbridge)
    # Fall back: add the library from the source tree (two levels up)
    set(_SHMBRIDGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
    if(EXISTS "${_SHMBRIDGE_ROOT}/shmbridge/CMakeLists.txt")
        add_subdirectory("${_SHMBRIDGE_ROOT}/shmbridge" shmbridge_ext EXCLUDE_FROM_ALL)
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../CMakeLists.txt")
        add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.." shmbridge_ext EXCLUDE_FROM_ALL)
    else()
        message(FATAL_ERROR
            "shmbridge not found. Either install it or build from the repo root.")
    endif()
endif()

# ── Pipeline nodes ────────────────────────────────────────────────────────────
foreach(node sensor_node planner_node control_node)
    add_executable(${node} ${node}.cpp)
    target_link_libraries(${node} PRIVATE shmbridge::shmbridge)
    target_compile_options(${node} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W3>
    )
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_libraries(${node} PRIVATE rt pthread)
    endif()
endforeach()
