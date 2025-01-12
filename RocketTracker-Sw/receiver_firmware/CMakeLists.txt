cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
# initialize the Raspberry Pi Pico SDK

project(pico_project C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../nanopb/extra)
find_package(Nanopb REQUIRED)
include_directories(${NANOPB_INCLUDE_DIRS})

set(PROTOS 
	${CMAKE_CURRENT_SOURCE_DIR}/../protocol/protocol.proto
)
nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(receiver_firmware
	src/main.cpp
	src/vgps.cpp
	src/comms/radio.cpp
	src/comms/comms_lib/frame_manager.cpp
	src/display/ssd1306.cpp
	src/usb_descriptors.cpp
	${PROTO_SRCS} ${PROTO_HDRS}
)

target_include_directories(receiver_firmware PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

pico_sdk_init()

target_link_libraries(receiver_firmware 
	pico_stdlib 
	pico_multicore 
	hardware_dma 
	hardware_spi 
	hardware_i2c 
	tinyusb_device 
	tinyusb_board
)

pico_enable_stdio_usb(receiver_firmware 1)
pico_enable_stdio_uart(receiver_firmware 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(receiver_firmware)
# rest of your project