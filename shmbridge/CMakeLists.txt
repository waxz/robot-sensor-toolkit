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
option(SHMBRIDGE_BUILD_TEST_MIGRATION
    "Build test_migration.cpp (node.hpp's phase-7 ring.hpp migration is done; see below)" ON)
# soak_test runs for 30+ minutes by default (design_ring_zero_copy.md
# §11.2/§15 phase 9) -- a manually-invoked validation tool, not something
# any routine build/test pass should run, hence its own option rather
# than folding it into SHMBRIDGE_BUILD_TESTS or SHMBRIDGE_BUILD_BENCH.
option(SHMBRIDGE_BUILD_SOAK_TEST
    "Build soak_test + soak_consumer_helper (30+ min multi-process validation run)" OFF)

# SHMBRIDGE_SANITIZE: empty (default) = no sanitizer; otherwise "address"
# (MSVC: AddressSanitizer only) or a GCC/Clang -fsanitize= argument, e.g.
# "address,undefined" or "thread" (WSL/native Linux). This project's Windows
# toolchain is MSVC (Visual Studio, see below) -- not MinGW, which has no
# sanitizer runtime at all -- so SHMBRIDGE_SANITIZE=address is expected to
# work on the same Windows host the plain build uses. See
# design_ring_zero_copy.md §15 phase 1 and §11.1 for why this is a required
# verification step, not an optional one, for Tier 1 concurrency-heavy code.
set(SHMBRIDGE_SANITIZE "" CACHE STRING
    "Sanitizer to build C++ unit tests with: empty, 'address,undefined', or 'thread'")

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeUtils.cmake")

message(STATUS "")
message(STATUS "==================== shmbridge build configuration ====================")
message(STATUS "  Compiler             : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Build type           : ${CMAKE_BUILD_TYPE}")
message(STATUS "  SIMD (-march=native) : ${SHMBRIDGE_NATIVE_ARCH}")
message(STATUS "  LTO                  : ${SHMBRIDGE_LTO}")
message(STATUS "  Futex notify (Linux) : ${SHMBRIDGE_NOTIFY}")
message(STATUS "  Sanitizer (tests)    : ${SHMBRIDGE_SANITIZE}")
message(STATUS "=========================================================================")
message(STATUS "")

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
# Cross-platform: core.hpp/ext_core.hpp/ext_topics.hpp all go through
# platform.hpp's shm/notify primitives (CreateFileMapping/MapViewOfFile/
# WaitOnAddress on Windows, shm_open/mmap/futex on Linux), same as ring.hpp/
# node.hpp/topic.hpp -- verified building and round-tripping on both native
# Windows/MSVC and WSL/Linux via `pip install -e .`. The only real gate here
# is whether pybind11 itself is found; no OS check is needed or present.
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
    message(STATUS "[shmbridge] pybind11 C++ extension '_core': ENABLED (will be built)")
else()
    # Only a problem for the Python package (src/shmbridge/__init__.py
    # imports _core unconditionally -- there is no pure-Python fallback;
    # see docs/design_ring_zero_copy.md and README.md's Build section).
    # Harmless for a pure-C++ consumer of this CMakeLists.txt (tests/
    # benchmarks/header-only usage never need pybind11 at all) -- not a
    # hard find_package(... REQUIRED) for that reason.
    message(STATUS "[shmbridge] pybind11 C++ extension '_core': SKIPPED "
                    "(pybind11 not found -- fine for a C++-only build of "
                    "this project, but the Python package requires it: "
                    "`import shmbridge` will fail without it)")
endif()

# ── C++ unit tests ────────────────────────────────────────────────────────────
# test_ring and test_migration are platform-independent (ring.hpp/node.hpp/
# registry.hpp all go through platform.hpp's cross-platform shm/notify
# primitives, no raw POSIX calls) and build on Windows, Linux, and macOS
# alike. test_topic (fork()-based) and test_core (a separate, not-yet-ported
# module) stay Unix-only.
if(SHMBRIDGE_BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()

    add_executable(test_ring tests/test_ring.cpp)
    target_link_libraries(test_ring PRIVATE shmbridge::shmbridge GTest::gtest_main)
    cmutils_target_set_warnings(test_ring MSVC /W4)
    cmutils_target_set_optimization(test_ring GNU_CLANG -O2 -g)

    if(SHMBRIDGE_SANITIZE)
        if(MSVC)
            # MSVC's /fsanitize= only implements AddressSanitizer (no UBSan,
            # no ThreadSanitizer) -- "thread" or "address,undefined" here
            # silently wouldn't do what the GCC/Clang branch below does, so
            # refuse rather than build something that looks sanitized but
            # isn't catching what the option name implies.
            if(NOT SHMBRIDGE_SANITIZE STREQUAL "address")
                message(FATAL_ERROR
                    "[shmbridge] SHMBRIDGE_SANITIZE='${SHMBRIDGE_SANITIZE}' requested "
                    "with MSVC, which only implements /fsanitize=address (no "
                    "ThreadSanitizer, no UndefinedBehaviorSanitizer) -- use "
                    "SHMBRIDGE_SANITIZE=address, or switch to a GCC/Clang toolchain "
                    "(WSL/Linux) for thread/undefined.")
            endif()
            # /fsanitize=address needs frame pointers and debug info the same
            # way the GCC/Clang path does; MSVC has no separate -O1-equivalent
            # worth forcing here (its /O2 default already keeps ASan's stack
            # traces usable, unlike GCC/Clang at -O2/-O3).
            # MSVC's /fsanitize=address forces static-CRT codegen regardless
            # of /MD/-MD requested elsewhere, so GTest must be built to match
            # (the default static-CRT GTest build, not -Dgtest_force_shared_
            # crt=ON) or linking fails with LNK2038 RuntimeLibrary mismatches.
            # _DISABLE_VECTOR_ANNOTATION/_DISABLE_STRING_ANNOTATION: ASan's
            # std::vector/std::string container-overflow annotations change
            # those types' ABI, so linking an ASan-compiled TU against a
            # library (GTest here) built without /fsanitize=address fails
            # with LNK2038 "annotate_string"/"annotate_vector" mismatches
            # unless annotations are disabled to match the unannotated
            # dependency -- Microsoft's own documented workaround for mixing
            # ASan and non-ASan code, not a local workaround invented here.
            target_compile_options(test_ring PRIVATE
                /fsanitize=address /Zi /MT
                -D_DISABLE_VECTOR_ANNOTATION -D_DISABLE_STRING_ANNOTATION)
            # No target_link_options() call here: MSVC's linker auto-detects
            # /INFERASANLIBS from the compile-time /fsanitize=address flag,
            # the way the `cl.exe /fsanitize=address ...` invocation this was
            # verified against does. The compiled binary's ASan runtime DLL
            # (clang_rt.asan_dynamic-x86_64.dll, in the same MSVC toolset
            # bin\Hostx64\x64 directory as cl.exe) must be resolvable via
            # PATH or sit next to the test_ring executable at run time, or
            # the exact wrong-version-loaded symptom is a binary that still
            # runs but silently isn't instrumented -- confirm with a known-bad
            # program (e.g. a deliberate one-past-the-end heap write) before
            # trusting a clean run, exactly as this verification did.
        else()
            if(NOT UNIX)
                message(WARNING
                    "[shmbridge] SHMBRIDGE_SANITIZE='${SHMBRIDGE_SANITIZE}' requested "
                    "with a non-MSVC Windows compiler (e.g. MinGW GCC), which has no "
                    "sanitizer runtime installed -- the link will fail. This project's "
                    "Windows toolchain is MSVC (Visual Studio): configure with that "
                    "generator/compiler for SHMBRIDGE_SANITIZE=address, or build on "
                    "WSL/Linux for thread/undefined, where GCC ships libasan/libtsan/"
                    "libubsan by default.")
            endif()
            # -fno-omit-frame-pointer: readable stack traces in sanitizer reports.
            # -O1 (not -O2/-O3): sanitizer diagnostics stay precise without being
            # so slow that a 200-trial RingLatencyDistribution run times out --
            # these builds are for correctness verification, not the latency
            # numbers phase1_ring_build_report.md is built from (those come from
            # an unsanitized build; sanitizer instrumentation itself perturbs
            # timing, so its latency numbers are never the ones reported there).
            target_compile_options(test_ring PRIVATE
                -fsanitize=${SHMBRIDGE_SANITIZE} -fno-omit-frame-pointer -O1 -g)
            target_link_options(test_ring PRIVATE -fsanitize=${SHMBRIDGE_SANITIZE})
        endif()
    endif()

    # --gtest_output=json: generates a machine-readable report (per-test
    # status, duration, and the latency properties test_ring.cpp records
    # via RecordProperty) that shmbridge/tests/phase1_ring_build_report.md
    # is built from, instead of hand-transcribing numbers off the console.
    # Sanitizer builds write to a separate file (see the if() above) so a
    # sanitizer run never overwrites the unsanitized timing report.
    if(SHMBRIDGE_SANITIZE)
        add_test(NAME test_ring COMMAND test_ring
                 "--gtest_output=json:${CMAKE_CURRENT_BINARY_DIR}/test_ring_sanitize_results.json")
    else()
        add_test(NAME test_ring COMMAND test_ring
                 "--gtest_output=json:${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ring_results.json")
    endif()

    # test_migration.cpp exercises node.hpp (and, through it, registry.hpp)
    # alongside ring.hpp/topic.hpp. registry.hpp used to call raw POSIX
    # shm_open/mmap/kill directly, making it (and therefore node.hpp and
    # this target) Unix-only; it's since been ported onto platform.hpp's
    # cross-platform primitives (shm_open_or_create/shm_unmap/current_pid/
    # process_alive), the same ones ring.hpp and topic.hpp already used --
    # see docs/design_ring_zero_copy.md §16 for why this was out of that
    # design plan's original scope and when it was ported. This target is
    # therefore built on every platform now, not just UNIX. The option
    # stays (default ON) as an easy way to exclude it without touching the
    # rest of the suite, not because it's expected to fail anywhere.
    if(SHMBRIDGE_BUILD_TEST_MIGRATION)
        add_executable(test_migration tests/test_migration.cpp)
        target_link_libraries(test_migration PRIVATE shmbridge::shmbridge GTest::gtest_main)
        cmutils_target_set_warnings(test_migration)
        cmutils_target_set_optimization(test_migration GNU_CLANG -O2 -g)
        add_test(NAME test_migration COMMAND test_migration)
    endif()

    # test_topic_sub_helper: a standalone subscriber process launched by
    # test_topic's MultipleSubscribers test via platform::run_and_capture()
    # to get genuine separate-OS-process concurrency without fork() (no
    # Windows equivalent). Built before test_topic so $<TARGET_FILE:...>
    # below resolves correctly.
    add_executable(test_topic_sub_helper tests/test_topic_sub_helper.cpp)
    target_link_libraries(test_topic_sub_helper PRIVATE shmbridge::shmbridge)
    cmutils_target_set_warnings(test_topic_sub_helper)
    cmutils_target_set_optimization(test_topic_sub_helper GNU_CLANG -O2 -g)

    # test_topic.cpp is platform-independent: every shm/notify/process
    # primitive it uses (directly, or via topic.hpp) goes through
    # platform.hpp, including the multi-process test above.
    add_executable(test_topic tests/test_topic.cpp)
    target_link_libraries(test_topic PRIVATE shmbridge::shmbridge GTest::gtest_main)
    add_dependencies(test_topic test_topic_sub_helper)
    cmutils_target_set_warnings(test_topic)
    cmutils_target_set_optimization(test_topic GNU_CLANG -O2 -g)
    target_compile_definitions(test_topic PRIVATE
        SHMBRIDGE_SUB_HELPER_PATH="$<TARGET_FILE:test_topic_sub_helper>"
    )
    add_test(NAME test_topic COMMAND test_topic)

    if(UNIX)
        # test_core.cpp tests core.hpp (ShmPublisher/ShmSubscriber), a
        # separate module from the ring/node/registry/topic work above; it
        # calls ::getpid() directly and links rt/pthread, and porting it is
        # a distinct, not-yet-scoped task.
        add_executable(test_core tests/test_core.cpp)
        target_link_libraries(test_core PRIVATE shmbridge::shmbridge GTest::gtest_main rt pthread)
        cmutils_target_set_warnings(test_core)
        cmutils_target_set_optimization(test_core GNU_CLANG -O2 -g)
        add_test(NAME test_core COMMAND test_core)
    endif()
endif()

# ── C++ microbenchmarks ───────────────────────────────────────────────────────
# bench_migration/bench_realistic exercise node.hpp the same way
# test_migration.cpp does and are equally portable now (platform::
# shm_destroy() replaced every direct ::shm_unlink() call) -- no longer
# UNIX-gated for that reason. SHMBRIDGE_BUILD_BENCH stays off by default
# on every platform; it was never UNIX-specific in intent, only in
# practice because these binaries wouldn't have compiled elsewhere before.
if(SHMBRIDGE_BUILD_BENCH)
    foreach(bench bench_migration bench_realistic)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/${bench}.cpp")
            add_executable(${bench} tests/${bench}.cpp)
            target_link_libraries(${bench} PRIVATE shmbridge::shmbridge)
            cmutils_target_set_optimization(${bench} GNU_CLANG -O3 -march=native -funroll-loops)
        endif()
    endforeach()
endif()

# ── Soak test (§11.2/§15 phase 9) ─────────────────────────────────────────────
if(SHMBRIDGE_BUILD_SOAK_TEST)
    add_executable(soak_consumer_helper tests/soak_consumer_helper.cpp)
    target_link_libraries(soak_consumer_helper PRIVATE shmbridge::shmbridge)
    cmutils_target_set_warnings(soak_consumer_helper)
    cmutils_target_set_optimization(soak_consumer_helper GNU_CLANG -O2 -g)

    add_executable(soak_test tests/soak_test.cpp)
    target_link_libraries(soak_test PRIVATE shmbridge::shmbridge)
    add_dependencies(soak_test soak_consumer_helper)
    cmutils_target_set_warnings(soak_test)
    cmutils_target_set_optimization(soak_test GNU_CLANG -O2 -g)
    target_compile_definitions(soak_test PRIVATE
        SHMBRIDGE_SOAK_CONSUMER_HELPER_PATH="$<TARGET_FILE:soak_consumer_helper>"
    )
    # Deliberately not add_test()'d -- see the option's own comment above.
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
