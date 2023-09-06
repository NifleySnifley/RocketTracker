CRC_POLY = 0x8d95

def crcb16(v):
    b = v << 8
    for i in range(8):
        if (b & 0x8000):
            b = (b<<1) ^ CRC_POLY
        else:
            b <<= 1
            
    return b & 0xFFFF

crctab = [crcb16(b) for b in range(256)]


testmsg = [1,2]
def crc16(bs):
    crc = 0
    for v in bs:
        prev = ((crc >> 8)  ^ v)
        crc = ((crc << 8) ^ crctab[prev]) & 0xFFFF
        
    return crc 

print(hex(crc16(testmsg)))
        
print("uint16_t CRC_TABLE[256] = {" + ",".join([hex(c) for c in crctab]) + "};\n")

print(hex(crcb16(0x01)))