target_include_directories(HC_SR04 INTERFACE include/hc_sr04)
zephyr_include_directories(include/hc_sr04)

add_subdirectory(src/hc_sr04)