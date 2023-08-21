import serial, time
dvc = serial.Serial('/dev/ttyACM0', 115200)

body = open("./testmessage.bin", 'rb').read()

# dvc.write(b"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0")

dvc.write(bytes([len(body)]))
dvc.write(body)
dvc.flush()
dvc.close()