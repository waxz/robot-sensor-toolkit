# CMakeUtils.cmake
#
# Generic, project-agnostic CMake helpers for cross-platform (GNU/Clang vs.
# MSVC, Linux vs. everything else) C++ targets. Nothing in this file
# references shmbridge, or any other specific project, package, or target
# name by name -- every project-specific value (package name, fallback
# path, dependency target, flag set) is passed in as an argument. This file
# can be copied as-is into any CMake project's cmake/ directory.
#
# See docs/cmake_best_practices.md (repo root) for the reasoning behind
# each pattern and when to reach for it.
#
# Functions:
#   cmutils_find_or_add_subdirectory(TARGET <tgt> PACKAGE <name>
#                                     SUBDIRECTORY <path> [BINARY_DIR <name>]
#                                     [CONFIG])
#   cmutils_target_set_warnings(<target> [GNU_CLANG <flags...>] [MSVC <flags...>])
#   cmutils_target_set_optimization(<target> [GNU_CLANG <flags...>] [MSVC <flags...>])
#   cmutils_target_link_if_platform(<target> <platform-name> <lib...>)
#   cmutils_add_executable(<target> SOURCES <src...> LINK_LIBRARIES <lib...>
#                           [GNU_CLANG_WARN <flags...>] [MSVC_WARN <flags...>]
#                           [LINUX_LIBS <lib...>])

# cmutils_find_or_add_subdirectory(TARGET <tgt> PACKAGE <name>
#                                   SUBDIRECTORY <path> [BINARY_DIR <name>]
#                                   [CONFIG])
#   The "installed package, else build it from a sibling source tree"
#   fallback used by any project that can be consumed either way. No-op if
#   <tgt> already exists (e.g. a parent project already brought it in).
#     TARGET       the imported/ALIAS target that must exist afterward
#                  (e.g. "shmbridge::shmbridge", "fmt::fmt").
#     PACKAGE      the name passed to find_package().
#     SUBDIRECTORY the source directory to add_subdirectory() if the
#                  package isn't found installed.
#     BINARY_DIR   the add_subdirectory() binary-dir name (default:
#                  "<package>_build"); only matters if it collides with
#                  another target in the same build.
#     CONFIG       forwarded to find_package() (CONFIG-mode package lookup
#                  instead of Module mode), when the package ships one.
function(cmutils_find_or_add_subdirectory)
    cmake_parse_arguments(ARG "CONFIG" "TARGET;PACKAGE;SUBDIRECTORY;BINARY_DIR" "" ${ARGN})
    if(NOT ARG_TARGET OR NOT ARG_PACKAGE OR NOT ARG_SUBDIRECTORY)
        message(FATAL_ERROR "cmutils_find_or_add_subdirectory requires TARGET, PACKAGE, and SUBDIRECTORY")
    endif()
    if(TARGET ${ARG_TARGET})
        return()
    endif()
    if(ARG_CONFIG)
        find_package(${ARG_PACKAGE} CONFIG QUIET)
    else()
        find_package(${ARG_PACKAGE} QUIET)
    endif()
    if(NOT TARGET ${ARG_TARGET})
        if(NOT ARG_BINARY_DIR)
            set(ARG_BINARY_DIR "${ARG_PACKAGE}_build")
        endif()
        if(NOT EXISTS "${ARG_SUBDIRECTORY}")
            message(FATAL_ERROR
                "${ARG_PACKAGE} not found installed, and the fallback source "
                "directory '${ARG_SUBDIRECTORY}' does not exist either -- "
                "install ${ARG_PACKAGE} or build from a source tree that has it.")
        endif()
        add_subdirectory("${ARG_SUBDIRECTORY}" "${ARG_BINARY_DIR}" EXCLUDE_FROM_ALL)
    endif()
endfunction()

# cmutils_target_set_warnings(<target> [GNU_CLANG <flags...>] [MSVC <flags...>])
#   Applies warning flags per compiler family via generator expressions.
#   Defaults to "-Wall;-Wextra" for GNU/Clang when GNU_CLANG isn't given;
#   MSVC gets nothing unless MSVC is given explicitly (MSVC's own default
#   warning level is left alone rather than silently changed).
function(cmutils_target_set_warnings target)
    cmake_parse_arguments(ARG "" "" "GNU_CLANG;MSVC" ${ARGN})
    if(NOT ARG_GNU_CLANG)
        set(ARG_GNU_CLANG -Wall -Wextra)
    endif()
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:${ARG_GNU_CLANG}>
    )
    if(ARG_MSVC)
        target_compile_options(${target} PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:${ARG_MSVC}>
        )
    endif()
endfunction()

# cmutils_target_set_optimization(<target> [GNU_CLANG <flags...>] [MSVC <flags...>])
#   Applies optimization flags per compiler family. Neither branch is set
#   unless given explicitly -- there is no sane cross-project default for
#   "how aggressively should this one target be optimized".
function(cmutils_target_set_optimization target)
    cmake_parse_arguments(ARG "" "" "GNU_CLANG;MSVC" ${ARGN})
    if(ARG_GNU_CLANG)
        target_compile_options(${target} PRIVATE
            $<$<CXX_COMPILER_ID:GNU,Clang>:${ARG_GNU_CLANG}>
        )
    endif()
    if(ARG_MSVC)
        target_compile_options(${target} PRIVATE
            $<$<CXX_COMPILER_ID:MSVC>:${ARG_MSVC}>
        )
    endif()
endfunction()

# cmutils_target_link_if_platform(<target> <platform-name> <lib...>)
#   Links <lib...> to <target> only when CMAKE_SYSTEM_NAME equals
#   <platform-name> (e.g. "Linux", "Darwin", "Windows").
function(cmutils_target_link_if_platform target platform_name)
    if(CMAKE_SYSTEM_NAME STREQUAL "${platform_name}")
        target_link_libraries(${target} PRIVATE ${ARGN})
    endif()
endfunction()

# cmutils_add_executable(<target> SOURCES <src...> LINK_LIBRARIES <lib...>
#                         [GNU_CLANG_WARN <flags...>] [MSVC_WARN <flags...>]
#                         [LINUX_LIBS <lib...>])
#   add_executable + target_link_libraries + cmutils_target_set_warnings +
#   an optional Linux-only link block, for the common "one source file (or
#   a few), one dependency set, standard warnings" executable target.
#   Omit GNU_CLANG_WARN/MSVC_WARN to get cmutils_target_set_warnings's own
#   defaults (GNU/Clang -Wall -Wextra, MSVC untouched).
function(cmutils_add_executable target)
    cmake_parse_arguments(ARG "" "" "SOURCES;LINK_LIBRARIES;GNU_CLANG_WARN;MSVC_WARN;LINUX_LIBS" ${ARGN})
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "cmutils_add_executable(${target} ...) requires SOURCES")
    endif()
    add_executable(${target} ${ARG_SOURCES})
    if(ARG_LINK_LIBRARIES)
        target_link_libraries(${target} PRIVATE ${ARG_LINK_LIBRARIES})
    endif()
    set(_warn_args "")
    if(ARG_GNU_CLANG_WARN)
        list(APPEND _warn_args GNU_CLANG ${ARG_GNU_CLANG_WARN})
    endif()
    if(ARG_MSVC_WARN)
        list(APPEND _warn_args MSVC ${ARG_MSVC_WARN})
    endif()
    cmutils_target_set_warnings(${target} ${_warn_args})
    if(ARG_LINUX_LIBS)
        cmutils_target_link_if_platform(${target} Linux ${ARG_LINUX_LIBS})
    endif()
endfunction()
