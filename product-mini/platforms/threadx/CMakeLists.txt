cmake_minimum_required(VERSION 3.8.2)

set (WAMR_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# Set the toolchain if not defined
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${WAMR_ROOT_DIR}/cmake/arm-gcc-cortex-m4.cmake")
endif()

include(${WAMR_ROOT_DIR}/cmake/utilities.cmake)

# find_package(ThreadX REQUIRED HINTS $ENV{THREADX_BASE})
project(wamr)

set (WAMR_BUILD_PLATFORM "threadx")

# Build as X86_32 by default, change to "AARCH64[sub]", "ARM[sub]", "THUMB[sub]", "MIPS" or "XTENSA"
# if we want to support arm, thumb, mips or xtensa
if (NOT DEFINED WAMR_BUILD_TARGET)
  set (WAMR_BUILD_TARGET "THUMBV7EM")
endif ()

if (NOT DEFINED WAMR_BUILD_INTERP)
  # Enable Interpreter by default
  set (WAMR_BUILD_INTERP 1)
endif ()

if (NOT DEFINED WAMR_BUILD_AOT)
  # Enable AOT by default.
  set (WAMR_BUILD_AOT 1)
endif ()

if (NOT DEFINED WAMR_BUILD_LIBC_BUILTIN)
  # Enable libc builtin support by default
  set (WAMR_BUILD_LIBC_BUILTIN 1)
endif ()

if (NOT DEFINED WAMR_BUILD_LIBC_WASI)
  # Disable libc wasi support by default
  set (WAMR_BUILD_LIBC_WASI 0)
endif ()

if (WAMR_BUILD_TARGET STREQUAL "RISCV64_LP64" OR WAMR_BUILD_TARGET STREQUAL "RISCV32_ILP32")
  set (WAMR_BUILD_FAST_INTERP 1)
endif ()

# Override the global heap usage
if (NOT DEFINED WAMR_BUILD_GLOBAL_HEAP_POOL)
  set (WAMR_BUILD_GLOBAL_HEAP_POOL 1)
endif ()

# Override the global heap size for small devices
if (NOT DEFINED WAMR_BUILD_GLOBAL_HEAP_SIZE)
  set (WAMR_BUILD_GLOBAL_HEAP_SIZE 131072) # 128 KB
endif ()

include (${WAMR_ROOT_DIR}/build-scripts/runtime_lib.cmake)
add_library(wamr ${WAMR_RUNTIME_LIB_SOURCE})

add_subdirectory(lib)
add_subdirectory(app)
