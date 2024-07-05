file(GLOB PB_SRCS compiled/*.pb.c)
idf_component_register(SRCS "fmgr.cpp" "../../../comms_lib/frame_manager.cpp" protobuf/pb_common.c protobuf/pb_decode.c protobuf/pb_encode.c ${PB_SRCS}
                    INCLUDE_DIRS "." "../../../comms_lib/" "compiled/" "protobuf/"
                    PRIV_REQUIRES driver)

include_directories("../../../comms_lib/")