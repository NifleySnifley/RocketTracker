all: test_encoder test_decoder
	./build/testencoder
	./build/testdecoder

test_encoder:
	gcc ./src/test/testencoder.cpp ./src/comms/frame_manager.cpp ./build/*.pb.c ../nanopb/*.c -I./build -I./src/comms -I../nanopb/ -o ./build/testencoder

test_decoder:
	gcc ./src/test/testdecoder.cpp ./src/comms/frame_manager.cpp ./build/*.pb.c ../nanopb/*.c -I./build -I./src/comms -I../nanopb/ -o ./build/testdecoder