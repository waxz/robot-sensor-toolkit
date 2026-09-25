cmake_minimum_required(VERSION 3.18)
project(shmbridge_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find shmbridge — either via installed CMake package or as a subdirectory
if(NOT TARGET shmbridge::shmbridge)
    find_package(shmbridge CONFIG QUIET)
endif()
if(NOT TARGET shmbridge::shmbridge)
    add_subdirectory(../.. shmbridge_build EXCLUDE_FROM_ALL)
endif()

add_executable(controller controller.cpp)
target_link_libraries(controller PRIVATE shmbridge::shmbridge)
target_compile_options(controller PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W3>
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(controller PRIVATE rt pthread)
endif()
