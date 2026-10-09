# jgb_rover motor / IMU board firmware for the Waveshare RP2040-Zero (Pico SDK 2.x).
#   cmake -B build -DPICO_SDK_PATH=/path/to/pico-sdk && cmake --build build -j
# -> build/jgb_rover_mcu.uf2 (copy to the RPI-RP2 drive, see README.md)
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(jgb_rover_mcu C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(PROTOCOL_DIR ${CMAKE_CURRENT_LIST_DIR}/../protocol)

add_executable(jgb_rover_mcu
  src/main.c
  src/control.c
  src/hw.c
  ${PROTOCOL_DIR}/rover_link.c
)
target_include_directories(jgb_rover_mcu PRIVATE src ${PROTOCOL_DIR})
target_compile_options(jgb_rover_mcu PRIVATE -Wall -Wextra -Wno-unused-parameter)

pico_generate_pio_header(jgb_rover_mcu ${CMAKE_CURRENT_LIST_DIR}/pio/quadrature_encoder.pio)
pico_generate_pio_header(jgb_rover_mcu ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

target_compile_definitions(jgb_rover_mcu PRIVATE
  # USB names: the udev rule (99-jgb-rover-mcu.rules) matches the product string
  USBD_MANUFACTURER="jgb_rover"
  USBD_PRODUCT="jgb_rover_mcu"
  # binary protocol: the firmware never blocks on a busy USB buffer (see link_send)
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)

target_link_libraries(jgb_rover_mcu
  pico_stdlib
  pico_unique_id
  hardware_pio
  hardware_pwm
  hardware_i2c
  hardware_adc
  hardware_watchdog
)

pico_enable_stdio_usb(jgb_rover_mcu 1)
pico_enable_stdio_uart(jgb_rover_mcu 0)
pico_add_extra_outputs(jgb_rover_mcu)
