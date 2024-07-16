cmake_minimum_required(VERSION 3.12)
project(rtrk_gui)
set(CMAKE_CXX_STANDARD 17)


find_package(OpenGL REQUIRED)

# add_subdirectory(external/ImFileDialog)
# add_subdirectory(external/implot)
# add_subdirectory(external/imgui)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/glfw)

# set(Boost_DEBUG 1)
find_package(Boost 1.80 REQUIRED system)

set(IMGUI_DIR external/imgui)
set(IMGUI_SOURCES  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

include_directories(menus)

add_executable(rtrk_gui main.cpp menus/link/link_menu.cpp ${IMGUI_SOURCES})
target_link_libraries(rtrk_gui PRIVATE glfw OpenGL::GL ${Boost_LIBRARIES})
target_include_directories(rtrk_gui PRIVATE ${Boost_INCLUDE_DIRS})