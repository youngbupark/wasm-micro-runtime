set (PLATFORM_SHARED_DIR ${CMAKE_CURRENT_LIST_DIR})

add_definitions(-DBH_PLATFORM_THREADX)

include_directories(${PLATFORM_SHARED_DIR})
include_directories(${PLATFORM_SHARED_DIR}/../include)
include_directories(${PLATFORM_SHARED_DIR}/threadx/ports/cortex_m4/gnu/inc)

include (${CMAKE_CURRENT_LIST_DIR}/../common/math/platform_api_math.cmake)

file (GLOB_RECURSE source_all ${PLATFORM_SHARED_DIR}/*.c)

set (PLATFORM_SHARED_SOURCE ${source_all} ${PLATFORM_COMMON_MATH_SOURCE})
