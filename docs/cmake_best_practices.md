# Guideline: CMake best practices for this repository

A reusable set of CMake conventions, applicable to any C++ subproject in
this repo (today: `shmbridge/`, `irsim_devices/cpp/`), not specific to
either one. Pair this with [`shmbridge/cmake/CMakeUtils.cmake`](../shmbridge/cmake/CMakeUtils.cmake),
a small, project-agnostic helper module that implements the patterns
below as callable functions — copy it into a new subproject's `cmake/`
directory rather than re-deriving these patterns from scratch.

**Contents**

- [1. Target-based design, not global flags](#1-target-based-design-not-global-flags)
- [2. Compiler-family branching: generator expressions, not `if(MSVC)`](#2-compiler-family-branching-generator-expressions-not-ifmsvc)
- [3. The "find installed, else build from source" pattern](#3-the-find-installed-else-build-from-source-pattern)
- [4. Platform-conditional linking](#4-platform-conditional-linking)
- [5. Options: naming, defaults, and `message(STATUS ...)`](#5-options-naming-defaults-and-messagestatus-)
- [6. Sanitizers: GCC/Clang and MSVC are not interchangeable](#6-sanitizers-gccclang-and-msvc-are-not-interchangeable)
- [7. Don't assume POSIX: `platform.hpp`-style portability](#7-dont-assume-posix-platformhpp-style-portability)
- [8. Install rules and `find_package()` consumers](#8-install-rules-and-find_package-consumers)
- [9. Using `CMakeUtils.cmake`](#9-using-cmakeutilscmake)
- [10. Checklist for a new CMakeLists.txt](#10-checklist-for-a-new-cmakeliststxt)

## 1. Target-based design, not global flags

Prefer `target_compile_options()`, `target_link_libraries()`,
`target_include_directories()`, and `target_compile_definitions()` over
their directory-scoped, flag-leaking equivalents (`add_compile_options()`,
`include_directories()`, `link_libraries()`). A flag set globally applies
to every target in the directory and everything added afterward,
including third-party code pulled in via `add_subdirectory()` — a
surprise for whoever adds the next dependency.

Express a library's own requirements (standard, optimization, platform
libraries) once on its own target and let `target_link_libraries()`
propagate them to consumers via `PUBLIC`/`INTERFACE` visibility, instead
of asking every consumer to repeat them. `shmbridge`'s own
`CMakeLists.txt` does this: the `shmbridge` INTERFACE target carries its
include path, `-O3 -march=native`/`/O2`, and `cxx_std_17` once; every
test, benchmark, and example target gets them for free by linking
`shmbridge::shmbridge`.

## 2. Compiler-family branching: generator expressions, not `if(MSVC)`

Use `$<$<CXX_COMPILER_ID:GNU,Clang>:...>` / `$<$<CXX_COMPILER_ID:MSVC>:...>`
generator expressions inside `target_compile_options()`, not
`if(MSVC) ... else() ... endif()` blocks that call `target_compile_options()`
twice. The generator-expression form:

- Works correctly with multi-config generators (Visual Studio, Xcode),
  where `if(MSVC)` is evaluated once at configure time but the actual
  compiler invocation can differ per build configuration.
- Composes: a single `target_compile_options()` call can carry both
  branches, keeping the flag set for one target in one place instead of
  split across two `if()` arms that can drift apart.

```cmake
target_compile_options(mytarget PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O2 -g -Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
```

`CMakeUtils.cmake`'s `cmutils_target_set_warnings()` and
`cmutils_target_set_optimization()` wrap exactly this pattern.

## 3. The "find installed, else build from source" pattern

A library that's sometimes consumed as an installed package and
sometimes as a sibling source directory (the common case for an example
program living inside the same repo as the library it demonstrates)
needs both lookup paths, attempted in a fixed order, with no duplicate
target error if both somehow succeed:

```cmake
if(NOT TARGET mylib::mylib)
    find_package(mylib CONFIG QUIET)
endif()
if(NOT TARGET mylib::mylib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. mylib_build EXCLUDE_FROM_ALL)
endif()
```

Key details, each a real bug this repo hit at some point:

- **Check `TARGET` before `find_package()`, not after.** A parent project
  that already built the library via `add_subdirectory()` higher up the
  tree will already have the target; re-running `find_package()` wastes a
  filesystem search and can find a *different*, stale installed copy.
- **`EXCLUDE_FROM_ALL`** on the fallback `add_subdirectory()` keeps
  `cmake --build .` from building the library's own tests/benchmarks when
  you only wanted the library itself for an example.
- **`QUIET`** on `find_package()` — a missing package here is an expected,
  handled case (the fallback), not something that should print a CMake
  warning before the fallback even runs.

`cmutils_find_or_add_subdirectory()` implements this, parameterized by
target name, package name, and fallback path, so the pattern is written
once instead of drifting across every example's `CMakeLists.txt` (which
is exactly what had happened here before consolidation — four copies,
three different fallback-path depths, one with an extra `elseif()`
branch that turned out to resolve to the same directory as the simple
case).

## 4. Platform-conditional linking

`CMAKE_SYSTEM_NAME STREQUAL "Linux"` (or `"Darwin"`, `"Windows"`) for
linking platform-specific libraries, not `UNIX`/`WIN32`/`APPLE` — those
three are broader than a single OS (`UNIX` is true on Linux *and* macOS
*and* WSL) and the wrong granularity when the two POSIX platforms need
different libraries (`rt` exists on Linux's glibc but not on macOS).

```cmake
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(mytarget PRIVATE rt pthread)
endif()
```

`cmutils_target_link_if_platform(<target> Linux rt pthread)` wraps this.

## 5. Options: naming, defaults, and `message(STATUS ...)`

- Prefix every `option()`/`set(... CACHE ...)` with the project name
  (`SHMBRIDGE_BUILD_TESTS`, not `BUILD_TESTS`) — an un-prefixed cache
  variable collides the moment two subprojects with their own test-build
  switch are configured in the same build tree.
- Default heavy or non-default-safe options (soak tests, sanitizers,
  benchmarks) to `OFF`; default options that only improve the common case
  (native-arch SIMD, LTO) to `ON`, and say why in the option's own help
  string, not just in a comment above it — `option()`'s second argument is
  what `cmake -LH` and `ccmake` actually show someone who didn't read the
  source.
- Print a short, grep-able build-configuration summary once, near the top
  of the top-level `CMakeLists.txt`, after the options are declared:

  ```cmake
  message(STATUS "  SIMD (-march=native) : ${SHMBRIDGE_NATIVE_ARCH}")
  ```

  This is what makes a CI log or a bug report's pasted configure output
  tell you what was actually built, instead of having to ask.

## 6. Sanitizers: GCC/Clang and MSVC are not interchangeable

Don't assume `-fsanitize=` and `/fsanitize=` are the same feature with
different spelling:

- MSVC's `/fsanitize=address` implements AddressSanitizer **only** — no
  ThreadSanitizer, no UndefinedBehaviorSanitizer. A sanitizer option that
  accepts `"thread"` or `"undefined"` needs to refuse those under MSVC
  with a `message(FATAL_ERROR ...)` that says so, rather than silently
  building an unsanitized binary that *looks* sanitized.
- MSVC's `/fsanitize=address` forces static-CRT codegen and ASan
  container-overflow ABI annotations regardless of what `/MD`/`/MT` was
  requested elsewhere. Anything it links against (e.g. GTest) must be
  built the same way, or you get `LNK2038` RuntimeLibrary/annotation
  mismatches that look like an unrelated linker bug. If you hit this: add
  `-D_DISABLE_VECTOR_ANNOTATION -D_DISABLE_STRING_ANNOTATION` to match an
  unannotated dependency (Microsoft's own documented workaround), and
  build the dependency with the default static-CRT settings, not
  `-Dgtest_force_shared_crt=ON`.
- A sanitizer build's binary needs the sanitizer runtime resolvable at
  run time (`clang_rt.asan_dynamic-x86_64.dll` next to the executable or
  on `PATH` for MSVC; `libasan`/`libtsan`/`libubsan` already present on
  mainstream Linux). Confirm an ASan build is actually instrumented with
  a known-bad program (a deliberate one-past-the-end write) before
  trusting a clean run — a wrong-version runtime can load silently and
  report nothing.
- Sanitizer instrumentation perturbs timing. Never source a project's
  reported latency/throughput numbers from a sanitized build; build it
  unsanitized for those, sanitized only for correctness verification, and
  keep the two runs' output files separate (e.g.
  `test_results.json` vs. `test_sanitize_results.json`) so one doesn't
  silently overwrite the other.

## 7. Don't assume POSIX: `platform.hpp`-style portability

Code meant to build on both MSVC and GCC/Clang cannot call
`::clock_gettime()`, `::nanosleep()`, `::shm_open()`, `::getpid()`, or
`::kill()` directly — route every OS primitive through one small
cross-platform header (this repo's convention:
[`shmbridge/include/shmbridge/platform.hpp`](../shmbridge/include/shmbridge/platform.hpp),
exposing `now_ns()`, `sleep_ns()`, `shm_open_or_create()`,
`current_pid()`, `process_alive()`, `run_and_capture()`) instead of
sprinkling `#ifdef _WIN32` through call sites.

This isn't a hypothetical: `bench_migration.cpp` and `bench_realistic.cpp`
in this repo had their own local `now_ns()`/`sleep_ns()` helpers built
directly on `::clock_gettime()`/`::nanosleep()`, which compiled fine on
WSL/GCC and failed outright on MSVC (`clock_gettime`: is not a member of
the global namespace) — caught only when a CMake refactor's verification
pass actually tried building them on both platforms. A target's
`CMakeLists.txt` entry can be byte-identical across platforms and the
*source* can still be the thing that isn't portable; verifying a
cross-platform claim means building the actual target on both platforms,
not inspecting its build file.

Platform differences to specifically watch for, learned in this repo:

- **Windows' `Sleep()`/`std::this_thread::sleep_for()` granularity is
  ~15.6 ms** — any code pacing itself with sub-15ms sleeps gets a much
  lower real rate on Windows than on Linux for the identical source
  (observed: ~20 kHz requested via 50 µs sleeps achieved ~5.9 kHz on WSL
  but only ~90 Hz on native Windows). If you need tight pacing, don't
  sleep at all in a tight loop (busy-spin, as this repo's benchmarks do)
  rather than relying on a sleep granularity assumption that holds on one
  platform and not the other.
- **WSL2's own VM can idle-teardown** a long-running background process
  even when nothing inside the WSL session or the Windows host is asleep
  — a process detached with `setsid`+`disown`+`nohup` and polled from
  outside is not the same as a process the OS guarantees to keep running.
  A multi-hour validation run on WSL needs to be the literal foreground
  process of a continuously-connected `wsl.exe` invocation (e.g. launched
  via Windows Task Scheduler), not something backgrounded and checked on
  periodically.

## 8. Install rules and `find_package()` consumers

For a header-only (or otherwise installable) library meant to be
consumed via `find_package()`, the minimum is:

```cmake
include(GNUInstallDirs)
install(TARGETS mylib EXPORT mylib-targets)
install(EXPORT mylib-targets
    FILE        mylib-targets.cmake
    NAMESPACE   mylib::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mylib
)
install(DIRECTORY include/mylib DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mylib-config-version.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/mylib-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/mylib-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mylib)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mylib-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mylib-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mylib)
```

with a `cmake/mylib-config.cmake.in` of just:

```cmake
@PACKAGE_INIT@
include("${CMAKE_CURRENT_LIST_DIR}/mylib-targets.cmake")
check_required_components(mylib)
```

This is what makes §3's `find_package(mylib CONFIG QUIET)` branch able to
succeed at all once the library is actually installed somewhere, rather
than the "find installed" half of that pattern being permanently
untested dead code.

## 9. Using `CMakeUtils.cmake`

Copy [`shmbridge/cmake/CMakeUtils.cmake`](../shmbridge/cmake/CMakeUtils.cmake)
into a new subproject's own `cmake/` directory (it has zero project-specific
references — every name is a parameter), then `include()` it and call:

```cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CMakeUtils.cmake")

cmutils_find_or_add_subdirectory(
    TARGET       mylib::mylib
    PACKAGE      mylib
    CONFIG
    SUBDIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
    BINARY_DIR   mylib_build
)

cmutils_add_executable(my_example
    SOURCES        my_example.cpp
    LINK_LIBRARIES mylib::mylib
    MSVC_WARN      /W3
    LINUX_LIBS     rt pthread
)
```

or, for a target needing more control than `cmutils_add_executable()`'s
single-call convenience (a test target also linking GTest, needing a
different warning level, or custom optimization flags):

```cmake
add_executable(my_test my_test.cpp)
target_link_libraries(my_test PRIVATE mylib::mylib GTest::gtest_main)
cmutils_target_set_warnings(my_test MSVC /W4)
cmutils_target_set_optimization(my_test GNU_CLANG -O2 -g)
```

See the module's own header comment for the full function list and
argument reference.

## 10. Checklist for a new CMakeLists.txt

- [ ] `target_*()` calls, not directory-scoped `add_compile_options()`/`include_directories()`.
- [ ] Compiler-family flags via `$<$<CXX_COMPILER_ID:...>:...>`, not `if(MSVC)`.
- [ ] Every `option()` is project-prefixed, defaults match its risk/cost, and the help string explains the default.
- [ ] Any "find installed, else build from source" logic uses `cmutils_find_or_add_subdirectory()` (or the pattern in §3) rather than a bespoke copy.
- [ ] Platform-specific linking checks `CMAKE_SYSTEM_NAME`, not `UNIX`/`WIN32` alone, when the two POSIX platforms diverge.
- [ ] No raw POSIX call (`clock_gettime`, `nanosleep`, `shm_open`, `getpid`, `kill`, ...) outside the project's one portability header.
- [ ] A claim that something is "cross-platform" is backed by an actual build on each claimed platform in this session — not inferred from the CMakeLists.txt alone.
- [ ] If the target is a sanitizer build: MSVC's sanitizer limitations (§6) are either irrelevant or explicitly handled, and the sanitizer run's output doesn't overwrite the unsanitized one's.
