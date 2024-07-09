mkdir -p ./lib/proto
protoc -I=../protocol/ --python_out=./lib/proto --pyi_out=./lib/proto ../protocol/protocol.proto
