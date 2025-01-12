cmake_minimum_required(VERSION 3.12)
project(rtrk_gui)
set(CMAKE_CXX_STANDARD 17)


find_package(OpenGL REQUIRED)


set(FRAME_MANAGER_DIR "../comms_lib/")
include_directories(${FRAME_MANAGER_DIR})

#add_subdirectory(external/latlon)
#add_subdirectory(external/ImOsm)

# add_subdirectory(external/ImFileDialog)
# add_subdirectory(external/implot)
# add_subdirectory(external/imgui)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

# set(Boost_DEBUG 1)
find_package(Boost 1.80 REQUIRED system)
find_package(absl REQUIRED)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
set(PROTO_SRCS ${CMAKE_CURRENT_LIST_DIR}/../protocol/protocol.proto)

INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER ${PROTO_SRCS})
ADD_LIBRARY(proto ${PROTO_HEADER} ${PROTO_SRC})

set(IMGUI_DIR external/imgui)
set(IMGUI_SOURCES  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

include_directories(menus)
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(MENU_SOURCES
    menus/connection_menu.cpp
    menus/debug_view_menu.cpp
    menus/data_viewer.cpp
    menus/map_menu.cpp
    menus/configurator.cpp
)

set(IMPLOT_SOURCES
    external/implot/implot.cpp
    external/implot/implot_items.cpp
    external/implot/implot_demo.cpp
)

add_library(implot ${IMPLOT_SOURCES})

add_executable(rtrk_gui main.cpp ${MENU_SOURCES} link/frame_manager_v2.cpp link/link.cpp ${IMGUI_SOURCES} ${PROTO_SRCS}  "include_imgui.h" "menus/map_menu.cpp")
# protobuf_generate(TARGET rtrk_gui)

target_link_libraries(rtrk_gui 
    glfw 
    OpenGL::GL 
    ${Boost_LIBRARIES} 
    ${Protobuf_LIBRARIES} 
    proto 
    implot 
    absl::log 
)
include_directories(
    ${Boost_INCLUDE_DIRS} 
    external/stb 
    external/implot 
    ${PROTOBUF_INCLUDE_DIRS}
)


add_custom_target(copy_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
)
add_dependencies(rtrk_gui copy_assets)