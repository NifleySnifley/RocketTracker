TARGET_HOST=seeker@192.168.50.164
# seeker@192.168.50.163
TARGET_PATH=/home/seeker/Documents/
TARGET_ARCH=arm-unknown-linux-gnueabihf
BUILT_PATH=./target/${TARGET_ARCH}/debug/seeker
BUILDARGS=
REMOTE_SCRIPT=./remotescript.sh

all: res deploy

dev: deploy exec_remote

build: fmt
	cross build ${BUILDARGS} --target ${TARGET_ARCH}

fmt: 
	rustfmt ./src/*.rs

deploy: build
	rsync -P ${BUILT_PATH} ${TARGET_HOST}:${TARGET_PATH}
	rsync -P ${REMOTE_SCRIPT} ${TARGET_HOST}:${TARGET_PATH}

exec_remote: 
	ssh -m hmac-sha1 -c aes128-ctr -C -X ${TARGET_HOST} "cd ${TARGET_PATH} && ./${REMOTE_SCRIPT}"

.PHONY: res
res:
	rsync -P ./res/* ${TARGET_HOST}:${TARGET_PATH}/res