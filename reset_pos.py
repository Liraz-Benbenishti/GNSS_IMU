# update_pos_velocity.py
in_file = "../data/drive_0708/gnss_1934_sf.pos"
out_file = "/app/gnss_1934_sf_reset.pos"

header_lines = []
data_lines = []

# 1️⃣ Read file
with open(in_file, "r") as f:
    for line in f:
        if line.startswith("%"):
            header_lines.append(line.rstrip("\n"))
        elif line.strip():
            data_lines.append(line.rstrip("\n"))

# 2️⃣ Identify if header has velocity columns
# just rebuild header (assumes columns already exist)
new_header = header_lines.copy()

# 3️⃣ Update data: set velocities to 0, velocity std to 30
new_data = []
for line in data_lines:
    # split by whitespace
    parts = line.split()
    # assume last 6 columns are: vn ve vu sdvn sdve sdvu
    if len(parts) < 6:
        raise ValueError("Unexpected number of columns in data line")
    # overwrite last 6 values
    parts[14:17] = ["0.0000", "0.0000", "0.0000"]  # vn, ve, vu
    parts[17:20] = ["30.0000", "30.0000", "30.0000"]  # sdvn, sdve, sdvu
    new_data.append("   ".join(parts))

# 4️⃣ Write output
with open(out_file, "w") as f:
    for h in new_header:
        f.write(h + "\n")
    for d in new_data:
        f.write(d + "\n")

print(f"Updated file saved as: {out_file}")
