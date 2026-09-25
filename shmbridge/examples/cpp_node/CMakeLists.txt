cmake_minimum_required(VERSION 3.18)
project(shmbridge_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find shmbridge — either via installed CMake package or as a subdirectory
if(NOT TARGET shmbridge::shmbridge)
    find_package(shmbridge CONFIG QUIET)
endif()
if(NOT TARGET shmbridge::shmbridge)
    # Fall back: source tree layout
    add_subdirectory(../.. shmbridge_build EXCLUDE_FROM_ALL)
endif()

foreach(ex talker listener)
    add_executable(${ex} ${ex}.cpp)
    target_link_libraries(${ex} PRIVATE shmbridge::shmbridge)
    target_compile_options(${ex} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W3>
    )
    # rt: POSIX real-time extensions (shm_open, clock_gettime) — Linux only
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_libraries(${ex} PRIVATE rt pthread)
    endif()
endforeach()
