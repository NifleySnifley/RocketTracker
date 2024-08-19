idf_component_register(SRCS "configuration.c"
                    INCLUDE_DIRS "." ${CMAKE_CURRENT_LIST_DIR}/../../../rtrk-config/generated
                    PRIV_REQUIRES driver nvs_flash esp_common)

file(GLOB CFGFILES ${CMAKE_CURRENT_LIST_DIR}/../../../rtrk-config/configfiles/*.jsonc)
add_custom_command(
    COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/../../../rtrk-config/configgen.py
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/../../../rtrk-config/configgen.py ${CFGFILES}
    OUTPUT ${CMAKE_CURRENT_LIST_DIR}/../../../rtrk-config/generated/config_generated.h
    COMMENT "Generating config"
)
set_property(SOURCE "configuration.c" APPEND PROPERTY OBJECT_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/../../../rtrk-config/generated/config_generated.h)