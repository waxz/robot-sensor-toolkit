cmake_minimum_required(VERSION 3.18)
project(lidar_embree LANGUAGES CXX)

# ── Python & pybind11 ─────────────────────────────────────────────────────────
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

# Try system pybind11 first; fall back to pip-installed pybind11.
find_package(pybind11 QUIET)
if (NOT pybind11_FOUND)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
        OUTPUT_VARIABLE _PB11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (_PB11_CMAKE_DIR)
        find_package(pybind11 REQUIRED HINTS "${_PB11_CMAKE_DIR}")
    else()
        message(FATAL_ERROR "pybind11 not found. Install: pip install pybind11")
    endif()
endif()

# ── Embree4 ───────────────────────────────────────────────────────────────────
# Ubuntu x86_64: apt install libembree-dev
# macOS:         brew install embree
# Windows x86_64 (Intel releases): set -DEMBREE_ROOT=<path to extracted zip>
#   cmake .. -DEMBREE_ROOT="C:/embree-4.3.0.x64.windows"

if (DEFINED EMBREE_ROOT)
    # Windows / manual install path
    set(EMBREE_INCLUDE_DIRS "${EMBREE_ROOT}/include")
    if (WIN32)
        set(EMBREE_LIBRARIES "${EMBREE_ROOT}/lib/embree4.lib")
    else()
        set(EMBREE_LIBRARIES "${EMBREE_ROOT}/lib/libembree4.so")
    endif()
    if (NOT EXISTS "${EMBREE_INCLUDE_DIRS}/embree4/rtcore.h")
        message(FATAL_ERROR "EMBREE_ROOT set to ${EMBREE_ROOT} but embree4/rtcore.h not found there.")
    endif()
else()
    # Standard system / pkg-config lookup
    find_package(embree 4.0 QUIET)
    if (NOT embree_FOUND)
        # Fallback: try pkg-config
        find_package(PkgConfig QUIET)
        if (PkgConfig_FOUND)
            pkg_check_modules(EMBREE REQUIRED embree4)
        else()
            message(FATAL_ERROR
                "Embree4 not found.\n"
                "  Ubuntu  : sudo apt install libembree-dev\n"
                "  macOS   : brew install embree\n"
                "  Windows : download from https://github.com/RenderKit/embree/releases\n"
                "            then pass -DEMBREE_ROOT=<path>")
        endif()
    endif()
endif()

# ── Build the extension module ────────────────────────────────────────────────
pybind11_add_module(lidar_embree MODULE lidar_embree.cpp)

target_compile_options(lidar_embree PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -march=native -ffast-math>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /arch:AVX2>
)

if (DEFINED EMBREE_ROOT)
    target_include_directories(lidar_embree PRIVATE "${EMBREE_INCLUDE_DIRS}")
    target_link_libraries(lidar_embree PRIVATE "${EMBREE_LIBRARIES}")
    # On Windows, copy the DLL next to the module
    if (WIN32)
        add_custom_command(TARGET lidar_embree POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                "${EMBREE_ROOT}/bin/embree4.dll"
                "$<TARGET_FILE_DIR:lidar_embree>"
            COMMENT "Copying embree4.dll"
        )
    endif()
elseif (embree_FOUND)
    target_include_directories(lidar_embree PRIVATE ${EMBREE_INCLUDE_DIRS})
    target_link_libraries(lidar_embree PRIVATE ${EMBREE_LIBRARIES})
else()
    # pkg-config path
    target_include_directories(lidar_embree PRIVATE ${EMBREE_INCLUDE_DIRS})
    target_link_libraries(lidar_embree PRIVATE ${EMBREE_LINK_LIBRARIES})
endif()

# ── Install into irsim_devices package ───────────────────────────────────────
install(TARGETS lidar_embree
    LIBRARY DESTINATION irsim_devices
    RUNTIME DESTINATION irsim_devices  # Windows DLL
)
