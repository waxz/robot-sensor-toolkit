cmake_minimum_required(VERSION 3.16)
project(cpp_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/CMakeUtils.cmake")

# ── Locate shmbridge ──────────────────────────────────────────────────────────
cmutils_find_or_add_subdirectory(
    TARGET       shmbridge::shmbridge
    PACKAGE      shmbridge
    CONFIG
    SUBDIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    BINARY_DIR   shmbridge_build
)

# ── Pipeline nodes ────────────────────────────────────────────────────────────
foreach(node sensor_node planner_node control_node)
    cmutils_add_executable(${node}
        SOURCES        ${node}.cpp
        LINK_LIBRARIES shmbridge::shmbridge
        MSVC_WARN      /W3
        LINUX_LIBS     rt pthread
    )
endforeach()
