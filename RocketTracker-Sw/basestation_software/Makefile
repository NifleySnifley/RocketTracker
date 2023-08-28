all: gen/mainwindow.py gen/protocol_pb2.py

gen/mainwindow.py: genfldr mainwindow.ui
	pyuic5 ./mainwindow.ui -o ./gen/mainwindow.py

gen/protocol_pb2.py: ../protocol/protocol.proto
	protoc -I=../protocol/ --python_out=./gen/ --pyi_out=./gen/ ../protocol/protocol.proto

genfldr:
	mkdir -p ./gen
