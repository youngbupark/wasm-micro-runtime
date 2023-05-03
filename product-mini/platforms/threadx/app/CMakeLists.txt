# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

include (${WAMR_ROOT_DIR}/build-scripts/runtime_lib.cmake)

set(SOURCES
    startup/${THREADX_TOOLCHAIN}/tx_initialize_low_level.S
    
    board_init.c
    console.c
    main.c
)

add_executable(${PROJECT_NAME}
    startup/${THREADX_TOOLCHAIN}/startup_stm32l4s5xx.s
    ${WAMR_RUNTIME_LIB_SOURCE}
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    azrtos::threadx
    azrtos::netxduo

    netx_driver
    stm32cubel4
)

target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        .
)

target_link_directories(${PROJECT_NAME}
    PUBLIC 
       startup/${THREADX_TOOLCHAIN}
)

if(CMAKE_C_COMPILER_ID STREQUAL "IAR")
    set_target_linker(${PROJECT_NAME} stm32l4s5xx_flash.icf)
else()
    set_target_linker(${PROJECT_NAME} STM32L4S5VITx_FLASH.ld)
endif()

post_build(${PROJECT_NAME})
