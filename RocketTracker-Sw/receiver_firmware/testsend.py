import serial
import time
import sys
dvc = serial.Serial('/dev/ttyACM0' if sys.platform ==
                    "linux" else "COM7", 115200)

datum_body = bytes([i for i in range(200)])
datum_type = 1  # INFO_Raw

# ID and CRC, (NOTE: CRC __WILL__ be invalid!)
# DatumInfo (type, length) type=6, length=6
# Datum protobuf
body = bytes.fromhex('00000000') + \
    int.to_bytes(datum_type, 1, byteorder='little') + int.to_bytes(len(datum_body), 2, byteorder='little') + \
    datum_body

print(len(body), body)
for x in range(100):
    print("sending!!!")
    dvc.write(b"\0\0")

    blen = int.to_bytes(len(body), 2, byteorder='little')
    data = blen + body

    for b in data:
        if (b == 0x00):
            dvc.write(bytes([0xFF, 0x02]))
        elif (b == 0xFF):
            dvc.write(bytes([0xFF, 0x01]))
        else:
            dvc.write(bytes([b]))
        dvc.flush()
        # time.sleep(0.01)
        dvc.read_all()

    dvc.write(b"\0\0")

    dvc.flush()
    time.sleep(0.5)

dvc.close()
