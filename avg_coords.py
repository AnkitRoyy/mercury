import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/home/soap/gps_filtered.txt'

lats, lons = [], []
with open(path) as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) >= 2:
            lats.append(float(parts[0]))
            lons.append(float(parts[1]))
if lats:
    print(f'Average Lat: {sum(lats)/len(lats):.8f}')
    print(f'Average Lon: {sum(lons)/len(lons):.8f}')
    print(f'Number of samples: {len(lats)}')