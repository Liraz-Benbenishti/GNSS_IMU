import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# -----------------------------
# Input file
# -----------------------------
pos_file = "/app/dodge/second_new.pos"

# -----------------------------
# Load RTKLIB POS file
# -----------------------------
times = []
lats = []
lons = []

with open(pos_file, "r") as f:
    for line in f:
        if line.startswith("%") or len(line.strip()) == 0:
            continue

        parts = line.split()

        # RTKLIB POS format:
        # 0: date, 1: time, 2: lat(deg), 3: lon(deg), 4: height(m)
        date_str = parts[0]
        time_str = parts[1]

        lat = float(parts[2])
        lon = float(parts[3])

        t = datetime.strptime(
            date_str + " " + time_str,
            "%Y/%m/%d %H:%M:%S.%f"
        )

        times.append(t)
        lats.append(lat)
        lons.append(lon)

times = np.array(times)
lats = np.array(lats)
lons = np.array(lons)

# -----------------------------
# Convert time to seconds
# -----------------------------
t0 = times[0]
t_sec = np.array([(t - t0).total_seconds() for t in times])

# -----------------------------
# Plot trajectory
# -----------------------------
plt.figure(figsize=(10, 8))

sc = plt.scatter(
    lons,
    lats,
    c=t_sec,
    s=5,
)

plt.plot(lons, lats, linewidth=0.5, alpha=0.6)

plt.xlabel("Longitude [deg]")
plt.ylabel("Latitude [deg]")
plt.title("Driving Trajectory (colored by time)")
plt.axis("equal")
plt.grid(True)

cbar = plt.colorbar(sc)
cbar.set_label("Time since start [s]")

plt.show()
