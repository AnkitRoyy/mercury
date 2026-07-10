import sys
lats, lons = [], []
with open('/home/$USER/gps_filtered.txt') as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) >= 2:
            lats.append(float(parts[0]))
            lons.append(float(parts[1]))
if lats:
    print(f'Average Lat: {sum(lats)/len(lats):.8f}')
    print(f'Average Lon: {sum(lons)/len(lons):.8f}')
    print(f'Number of samples: {len(lats)}')

