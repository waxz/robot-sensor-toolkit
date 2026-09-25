cmake_minimum_required(VERSION 3.18)
project(shmbridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Feature flags ─────────────────────────────────────────────────────────────
option(SHMBRIDGE_NATIVE_ARCH "Build with -march=native (SIMD memcpy, optimal pause)"  ON)
option(SHMBRIDGE_LTO         "Enable link-time optimisation on the INTERFACE target"   ON)
option(SHMBRIDGE_NOTIFY      "Enable futex broadcast notify (Linux only)"              ON)
option(SHMBRIDGE_BUILD_TESTS "Build C++ unit tests (requires googletest)"              OFF)
option(SHMBRIDGE_BUILD_BENCH "Build C++ microbenchmarks"                               OFF)

# ── shmbridge INTERFACE library ───────────────────────────────────────────────
# Header-only: consumers link against shmbridge::shmbridge and get all include
# paths, compile options, and preprocessor flags propagated automatically.

add_library(shmbridge INTERFACE)
add_library(shmbridge::shmbridge ALIAS shmbridge)

target_include_directories(shmbridge INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(shmbridge INTERFACE cxx_std_17)

# Optimisation options propagated to every TU that links shmbridge
target_compile_options(shmbridge INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
        -O3
        $<$<BOOL:${SHMBRIDGE_NATIVE_ARCH}>:-march=native>
        -funroll-loops
    >
    $<$<CXX_COMPILER_ID:MSVC>:/O2>
)

# LTO: set the property; consumer project must also enable it for full effect
if(SHMBRIDGE_LTO)
    set_target_properties(shmbridge PROPERTIES
        INTERFACE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Preprocessor flags
target_compile_definitions(shmbridge INTERFACE
    $<$<BOOL:${SHMBRIDGE_NOTIFY}>:SB_ENABLE_NOTIFY=1>
    $<$<NOT:$<BOOL:${SHMBRIDGE_NOTIFY}>>:SB_ENABLE_NOTIFY=0>
)

# Windows: link Synchronization.lib for WaitOnAddress / WakeByAddressAll
target_link_libraries(shmbridge INTERFACE
    $<$<PLATFORM_ID:Windows>:Synchronization>
)

# ── pybind11 extension module ─────────────────────────────────────────────────
# POSIX-only: shm_open / mmap are not available on Windows, so the extension
# is only built on Unix-like systems where the underlying APIs exist.
find_package(pybind11 CONFIG QUIET)

if(pybind11_FOUND)
    pybind11_add_module(_core csrc/py_bindings.cpp)
    target_include_directories(_core PRIVATE include)
    target_compile_options(_core PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:
            -O3
            $<$<BOOL:${SHMBRIDGE_NATIVE_ARCH}>:-march=native>
            -fvisibility=hidden
        >
        $<$<CXX_COMPILER_ID:MSVC>:/O2>
    )
    # Windows: WaitOnAddress / WakeByAddressAll from platform.hpp
    target_link_libraries(_core PRIVATE
        $<$<PLATFORM_ID:Windows>:Synchronization>
        $<$<PLATFORM_ID:Linux>:rt>
    )
    install(TARGETS _core DESTINATION shmbridge)
endif()

# ── C++ unit tests ────────────────────────────────────────────────────────────
# Tests and benchmarks use POSIX SHM; only build on Unix.
if(UNIX AND SHMBRIDGE_BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()

    add_executable(test_topic tests/test_topic.cpp)
    target_link_libraries(test_topic PRIVATE shmbridge::shmbridge GTest::gtest_main)
    target_compile_options(test_topic PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-O2 -g -Wall -Wextra>
    )
    add_test(NAME test_topic COMMAND test_topic)

    add_executable(test_migration tests/test_migration.cpp)
    target_link_libraries(test_migration PRIVATE shmbridge::shmbridge GTest::gtest_main)
    target_compile_options(test_migration PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-O2 -g -Wall -Wextra>
    )
    add_test(NAME test_migration COMMAND test_migration)

    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE shmbridge::shmbridge GTest::gtest_main rt pthread)
    target_compile_options(test_core PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-O2 -g -Wall -Wextra>
    )
    add_test(NAME test_core COMMAND test_core)
endif()

# ── C++ microbenchmarks ───────────────────────────────────────────────────────
if(UNIX AND SHMBRIDGE_BUILD_BENCH)
    foreach(bench bench_migration bench_realistic)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/${bench}.cpp")
            add_executable(${bench} tests/${bench}.cpp)
            target_link_libraries(${bench} PRIVATE shmbridge::shmbridge)
            target_compile_options(${bench} PRIVATE
                $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -march=native -funroll-loops>
            )
        endif()
    endforeach()
endif()

# ── Install rules for the INTERFACE target ────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS shmbridge EXPORT shmbridge-targets)
install(EXPORT shmbridge-targets
    FILE        shmbridge-targets.cmake
    NAMESPACE   shmbridge::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/shmbridge
)
install(DIRECTORY include/shmbridge
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/shmbridge-config-version.cmake"
    VERSION       3.0.0
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/shmbridge-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/shmbridge-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/shmbridge
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/shmbridge-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/shmbridge-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/shmbridge
)
