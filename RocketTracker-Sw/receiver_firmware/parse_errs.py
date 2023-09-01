from de_esc import de_esc
import colorama

log = list(open("./logrx.log", 'rb').read())

# print(log)

def split_els(arr: list, el):
	if (len(arr) == 0 or (len(arr) == 1 and arr[0] == el)):
		return []
	subs = [[]]
	for e in arr:
		if (e == el):
			if (len(subs[-1])):
				subs.append([])
		else:
			subs[-1].append(e)
	return subs

messages = split_els(log, 0)

# print([len(a) for a in messages])
bads = 0
for m in messages:
	if (len(m) == 0):
		print("Null packet!")
		continue

	bad = False
	m = de_esc(m)
	mlen = m[0]
	act_len=len(m)-1

	if (act_len == 14):
		continue # Don't worry about the rx status messages

	print(f"Message len {m[0]}, {act_len}")
	print(f"\t {m[:10]}")

	if (act_len != mlen):
		print(f"\tIndicated and real message len mismatch:\n\t\tindicated: {mlen}, real: {act_len}")
		bad = True
		
	else:
		if (mlen == 200):
			if not all([m[i+1] == i for i in range(200)]):
				print(f"\tInvalid data!")
				bad = True

	if (bad):
		bads += 1
		print(colorama.Fore.RED + "BAD!" + colorama.Fore.RESET)
	else:
		print(colorama.Fore.GREEN + "GOOD!" + colorama.Fore.RESET)

	print('\n')
 
print(f"{(bads/len(messages)) * 100.0}% bad out of {len(messages)}")