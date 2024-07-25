file(GLOB_RECURSE files "*.c")

idf_component_register(SRCS ${files}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver)