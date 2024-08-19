file(GLOB DD_SRCS ${CMAKE_CURRENT_LIST_DIR}/dd_board/*.c)
idf_component_register(SRCS "alt_filter.cpp" "configutils.c" "logging.c" "main.c" "logging.c" "sensors.c" "prelog.c" "alt_filter.cpp" ${DD_SRCS}
                    INCLUDE_DIRS "." "../../rtrk-config/generated" "./dd_board/")
