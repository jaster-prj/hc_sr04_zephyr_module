set(HC_SR04_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory_ifdef(CONFIG_HC_SR04 src)