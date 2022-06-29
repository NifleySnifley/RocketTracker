TARGET_HOST=seeker@192.168.50.164
# seeker@192.168.50.163
TARGET_PATH=/home/seeker/Documents/
TARGET_ARCH=arm-unknown-linux-gnueabihf
BUILT_PATH=./target/${TARGET_ARCH}/debug/seeker
BUILDARGS=
REMOTE_SCRIPT=./remotescript.sh

all: deploy

dev: all exec_remote

build:
	cross build ${BUILDARGS} --target ${TARGET_ARCH}

deploy: build
	rsync -P ${BUILT_PATH} ${TARGET_HOST}:${TARGET_PATH}
	rsync -P ${REMOTE_SCRIPT} ${TARGET_HOST}:${TARGET_PATH}

exec_remote: 
	ssh -m hmac-sha1 -c aes128-ctr -C -X ${TARGET_HOST} "${TARGET_PATH}/${REMOTE_SCRIPT}"
