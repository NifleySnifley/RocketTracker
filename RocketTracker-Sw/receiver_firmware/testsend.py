import serial, time
dvc = serial.Serial('/dev/ttyACM0', 115200)

body = open("./testmessage.bin", 'rb').read()

dvc.write(b"\0\0\0\0\0\0\0\0\0\0")

dvc.write(bytes([len(body)]))

for b in body:
    if (b == 0x00):
        dvc.write(bytes([0xFF, 0x02]))
    elif (b == 0xFF):
        dvc.write(bytes([0xFF, 0x01]))
    else:
        dvc.write(bytes([b]))
        
dvc.flush()
dvc.close()