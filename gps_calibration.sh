#!/usr/bin/env bash

set -e

GPS_LOG="$HOME/gps_log.txt"
GPS_FILTERED="$HOME/gps_filtered.txt"
AVG_SCRIPT="/home/soap/probes/mercury/avg_coords.py"

echo "Checking GPS fix..."
ros2 topic echo /gps --once || true

read -p "Does status show 0 or greater? (y/n): " ans

if [[ "$ans" != "y" ]]; then
    echo "GPS has no fix. Move outside and try again."
    exit 1
fi

echo
echo "Collecting GPS samples for 100 seconds..."
timeout 60 ros2 topic echo /gps > "$GPS_LOG" || true

echo "Filtering FIX samples..."

awk '
/latitude:/ {lat=$2}
/longitude:/ {lon=$2}
/status:/ {getline; s=$2}
/---/{
    if(s>=0)
        print lat, lon
}
' "$GPS_LOG" > "$GPS_FILTERED"

COUNT=$(wc -l < "$GPS_FILTERED")

echo "Valid samples: $COUNT"

if [ "$COUNT" -eq 0 ]; then
    echo "No valid FIX samples found."
    exit 1
fi

echo
echo "Filtered samples:"
cat "$GPS_FILTERED"

echo
read -p "Enter TRUE_HEADING_DEG: " TRUE_HEADING

YAW_OFFSET=$(python3 - <<EOF
import math
print((90-$TRUE_HEADING)*math.pi/180)
EOF
)

# call avg_coords.py ONCE, capture output, parse both values from it
AVG_OUT=$(python3 "$AVG_SCRIPT" "$GPS_FILTERED")
echo "$AVG_OUT"

LAT=$(echo "$AVG_OUT" | awk '/Average Lat/ {print $3}')
LON=$(echo "$AVG_OUT" | awk '/Average Lon/ {print $3}')

echo
echo "===================================="
echo "Use these values:"
echo
echo "MEASURED_LAT    = $LAT"
echo "MEASURED_LON    = $LON"
echo "TRUE_HEADING    = $TRUE_HEADING"
echo "YAW_OFFSET_RAD  = $YAW_OFFSET"
echo
echo "datum: [$LAT, $LON, $YAW_OFFSET]"
echo "===================================="