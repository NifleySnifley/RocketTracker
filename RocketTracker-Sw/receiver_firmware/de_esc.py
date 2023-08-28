inp = "01 7D 00 0E FF 02 FF 02 40 0A 0D FF 02 FF 02 FF \
02 FF 02 15 FF 02 FF 02 FF 02 FF 02 00"

bs = [int(i, 16) for i in inp.split()]

print([hex(b)for b in bs])

i = 0
while (i < len(bs)):
	if (bs[i] == 0xFF):
		i += 1
		if (bs[i] == 0x02):
			print(hex(0x00), end=' ')
		elif (bs[i] == 0x01):
			print(hex(0xFF), end=' ')
		else:
			print(f"Invalid escape sequence at byte {i-1}!")
			break
	else:
		print(hex(bs[i]), end=' ')
	i += 1
        
print()