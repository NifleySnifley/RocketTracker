file(GLOB_RECURSE files "*.cpp")

idf_component_register(SRCS ${files}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver eigen)