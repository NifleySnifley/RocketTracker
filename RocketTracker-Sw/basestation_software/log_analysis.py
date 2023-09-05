import json
import matplotlib.pyplot as plt
import sys
from io import TextIOWrapper


def getLatLons(log: TextIOWrapper):
    assert log.readable()
    loglines = [json.loads(l.strip()) for l in f.readlines()]

    latlons = []

    for datum in loglines:
        if datum["type"] == "TLM_GPS_Info" and all([p in datum["datum"].keys() for p in ['lat', 'lon']]):
            # print(datum["datum"].keys())
            latlons.append((
                datum["datum"]["lat"],
                datum["datum"]["lon"],
            ))

    return latlons


if __name__ == "__main__":
    assert len(sys.argv) > 1

    latlons = []
    with open(sys.argv[1], 'r') as f:
        latlons = getLatLons(f)

    # Dumb filtering
    latlons = [(la, lo)
               for la, lo in latlons if (40 < la < 50 and -90 < lo < -80)]

    for (lat, lon) in latlons:
        print(lat, lon)
    plt.plot([-c[0] for c in latlons], [c[1] for c in latlons])
    plt.show()
