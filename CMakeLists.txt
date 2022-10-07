cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(thermal-camera C CXX ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories("st7789-library-for-pico/include" "mlx90640-RPI-Pico/include")

add_executable(thermal-camera src/main.cpp st7789-library-for-pico/st7789.cpp mlx90640-RPI-Pico/MLX90640_API.cpp mlx90640-RPI-Pico/MLX90640_PICO_I2C_Driver.cpp)

target_link_libraries(thermal-camera pico_stdlib hardware_i2c hardware_spi)

pico_add_extra_outputs(thermal-camera)

#pico_enable_stdio_usb(mlx90640-RPI-Pico 1)
#pico_enable_stdio_uart(mlx90640-RPI-Pico 0)

