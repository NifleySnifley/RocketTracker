cmake_minimum_required(VERSION 3.12)
project(rtrk_gui)
set(CMAKE_CXX_STANDARD 17)


find_package(OpenGL REQUIRED)

set(FRAME_MANAGER_DIR "../comms_lib/")
include_directories(${FRAME_MANAGER_DIR})

# add_subdirectory(external/ImFileDialog)
# add_subdirectory(external/implot)
# add_subdirectory(external/imgui)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

# set(Boost_DEBUG 1)
find_package(Boost 1.80 REQUIRED system)

# include(FindProtobuf)
# find_package(Protobuf REQUIRED)
# include_directories(${PROTOBUF_INCLUDE_DIR})
# include_directories(${PROTOBUF_INCLUDE_DIRS})
set(PROTO_SRCS ${CMAKE_CURRENT_LIST_DIR}/../protocol/protocol.proto)
# add_library(proto ${PROTO_HEADER} ${PROTO_SRC})
# include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(external/protobuf)


set(IMGUI_DIR external/imgui)
set(IMGUI_SOURCES  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

include_directories(menus)
include_directories(${CMAKE_CURRENT_LIST_DIR})

set(MENU_SOURCES
    menus/connection_menu.cpp
    menus/debug_view_menu.cpp
)

add_executable(rtrk_gui main.cpp ${MENU_SOURCES} link/frame_manager_v2.cpp link/link.cpp ${IMGUI_SOURCES} ${PROTO_SRCS})
protobuf_generate(TARGET rtrk_gui)

target_link_libraries(rtrk_gui glfw OpenGL::GL ${Boost_LIBRARIES} ${Protobuf_LIBRARIES} proto)
target_include_directories(rtrk_gui ${Boost_INCLUDE_DIRS})


add_custom_target(copy_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
)
add_dependencies(rtrk_gui copy_assets)