
cmake_minimum_required(VERSION 3.13)

set (PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)

# define project
project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # LSP

pico_sdk_init()

file(GLOB_RECURSE SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/BNO08x/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/BNO08x/*.c
)

add_executable(main
    src/main.cpp
    ${SOURCES}
)

# Add include directory
target_include_directories(main
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/BNO08x
)

# generate PIO
pico_generate_pio_header(
    main
    ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio
)

pico_enable_stdio_usb(main 0) # disable USB
pico_enable_stdio_uart(main 1) # enable UART

pico_add_extra_outputs(main)
target_link_libraries(main
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_i2c
    hardware_spi
    hardware_pwm
)
