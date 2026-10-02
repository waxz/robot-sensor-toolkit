cmake_minimum_required(VERSION 3.18)
project(shmbridge_ring_features_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/CMakeUtils.cmake")

cmutils_find_or_add_subdirectory(
    TARGET       shmbridge::shmbridge
    PACKAGE      shmbridge
    CONFIG
    SUBDIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    BINARY_DIR   shmbridge_build
)

cmutils_add_executable(ring_features_demo
    SOURCES        ring_features_demo.cpp
    LINK_LIBRARIES shmbridge::shmbridge
    MSVC_WARN      /W3
    LINUX_LIBS     rt pthread
)
