import numpy as np
from datetime import datetime

in_file = "/app/gnss_gili_first_headless.pos"
out_file = "/app/gnss_gili_first_headless_w_inter_vel.pos"

# -----------------------------
# WGS84 constants
# -----------------------------
A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)

def llh_to_ecef(lat, lon, h):
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)
    N = A / np.sqrt(1 - E2 * np.sin(lat)**2)
    return np.array([
        (N + h) * np.cos(lat) * np.cos(lon),
        (N + h) * np.cos(lat) * np.sin(lon),
        (N * (1 - E2) + h) * np.sin(lat)
    ])

def ecef_to_enu_matrix(lat0, lon0):
    lat0 = np.deg2rad(lat0)
    lon0 = np.deg2rad(lon0)
    return np.array([
        [-np.sin(lon0),              np.cos(lon0),               0],
        [-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)],
        [ np.cos(lat0)*np.cos(lon0),  np.cos(lat0)*np.sin(lon0), np.sin(lat0)]
    ])

# -----------------------------
# Read file
# -----------------------------
header_lines = []
rows = []

with open(in_file, "r") as f:
    for line in f:
        if line.startswith("%"):
            header_lines.append(line.rstrip())
        elif line.strip():
            rows.append(line.split())

# -----------------------------
# Parse time & position
# -----------------------------
times = []
llh = []

for r in rows:
    t = datetime.strptime(
        r[0] + " " + r[1],
        "%Y/%m/%d %H:%M:%S.%f"
    ).timestamp()
    times.append(t)
    llh.append([float(r[2]), float(r[3]), float(r[4])])

times = np.array(times)
llh = np.array(llh)

# Stop if NaNs exist
if np.isnan(llh).any() or np.isnan(times).any():
    raise ValueError("NaNs detected in input data")

# -----------------------------
# Convert positions to ECEF
# -----------------------------
ecef = np.array([llh_to_ecef(*p) for p in llh])

# Fixed ENU reference (first epoch)
R_enu = ecef_to_enu_matrix(llh[0, 0], llh[0, 1])

# -----------------------------
# Central-difference velocity
# -----------------------------
vel_enu = np.zeros((len(ecef), 3)) 

for i in range(1, len(ecef) - 1):
    dt = times[i + 1] - times[i - 1]
    if dt <= 0:
        continue
    v_ecef = (ecef[i + 1] - ecef[i - 1]) / dt
    vel_enu[i] = R_enu @ v_ecef

# Copy edge values
vel_enu[0] = vel_enu[1]
vel_enu[-1] = vel_enu[-2]

# -----------------------------
# Velocity STD estimation
# -----------------------------
# Typical RTKLIB position STD (meters)
sigma_p_ne = 0.05   # horizontal
sigma_p_u  = 0.10   # vertical

dt_mean = np.mean(np.diff(times))

sigma_v_ne = np.sqrt(2) * sigma_p_ne / dt_mean
sigma_v_u  = np.sqrt(2) * sigma_p_u  / dt_mean

# -----------------------------
# Rewrite file
# -----------------------------
new_header = []
for h in header_lines:
    if "latitude(deg)" in h:
        h += (
            "   vn(m/s)   ve(m/s)   vu(m/s)"
            "   sdvn(m/s)   sdve(m/s)   sdvu(m/s)"
        )
    new_header.append(h)

new_data = []
for i, r in enumerate(rows):
    new_data.append(
        " ".join(r)
        + f"   {vel_enu[i,0]:.4f}   {vel_enu[i,1]:.4f}   {vel_enu[i,2]:.4f}"
        + f"   {sigma_v_ne:.3f}   {sigma_v_ne:.3f}   {sigma_v_u:.3f}"
    )

with open(out_file, "w") as f:
    for h in new_header:
        f.write(h + "\n")
    for d in new_data:
        f.write(d + "\n")

print(f"Written: {out_file}")
