
set(DRV2605_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "DRV2605 module root directory")

# Subdirectories
add_subdirectory(drivers)

# Include headers
zephyr_include_directories(drivers)