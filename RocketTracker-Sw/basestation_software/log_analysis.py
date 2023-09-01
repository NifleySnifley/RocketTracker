import json
import matplotlib.pyplot as plt
import sys

assert len(sys.argv) > 1

loglines = []
with open(sys.argv[1], 'r') as f:
    loglines = [json.loads(l.strip()) for l in f.readlines()]

latlons = []

for datum in loglines:
    if datum["type"] == "TLM_GPS_Info" and all([p in datum["datum"].keys() for p in ['lat', 'lon']]):
        # print(datum["datum"].keys())
        print((
            datum["datum"]["lat"],
            datum["datum"]["lon"],
        ))

plt.plot([c[0] for c in latlons], [c[1] for c in latlons])
plt.show()
