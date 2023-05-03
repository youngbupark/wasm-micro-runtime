
# Define ThreadX user configuration
set(TX_USER_FILE "${CMAKE_CURRENT_LIST_DIR}/config/tx_user.h" CACHE STRING "Enable TX user configuration")

# Define NetXDuo user configuration
set(NX_USER_FILE "${CMAKE_CURRENT_LIST_DIR}/config/nx_user.h" CACHE STRING "Enable NX user configuration")

set(NXD_ENABLE_FILE_SERVERS OFF CACHE BOOL "Disable fileX dependency by netxduo")

# Core libraries
add_subdirectory(threadx)
add_subdirectory(netxduo)

add_subdirectory(stm32cubel4)
add_subdirectory(netx_driver)