import serial, time
import sys
dvc = serial.Serial('/dev/ttyACM0' if sys.platform == "linux" else "COM15", 115200)

# dvc.write(bytes([0,1,2,0]))


body = bytes([i for i in range(200)]) #open("./testmessage.bin", 'rb').read()

print(body)
for x in range(100):
	dvc.write(b"\0\0")

	dvc.write(bytes([len(body)]))

	for b in body:
		if (b == 0x00):
			dvc.write(bytes([0xFF, 0x02]))
		elif (b == 0xFF):
			dvc.write(bytes([0xFF, 0x01]))
		else:
			dvc.write(bytes([b]))
		dvc.flush()
		# time.sleep(0.01)

			
	dvc.write(b"\0\0")

	dvc.flush()
	time.sleep(0.5)
 
dvc.close()
