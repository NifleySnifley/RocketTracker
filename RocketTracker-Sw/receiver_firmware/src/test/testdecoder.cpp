#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "frame_manager.h"

void read_datum(int i, MessageTypeID id, int len, uint8_t* data) {
	printf("Datum ID %d of length %d\n", (int)id, len);
	printf("\t");
	for (int i = 0; i < len; ++i)
		printf("%.2x ", data[i]);
	printf("\n");
}

int main(int argc, char const* argv[]) {
	FrameManager f;

	FILE* file = fopen("./testmessage.bin", "rb");
	fseek(file, 0L, SEEK_END);
	int flen = ftell(file);
	rewind(file);

	uint8_t* data = (uint8_t*)malloc(flen);
	assert(flen == fread(data, 1, flen, file));

	fclose(file);

	f.load_frame(data, flen);
	f.decode_frame(read_datum);

	return 0;
}
