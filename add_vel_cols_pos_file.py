import pathlib

in_file = "/app/gnss_gili_first_headless.pos"
out_file = "/app/gnss_gili_first_headless_w_vel.pos"

header_lines = []
data_lines = []

with open(in_file, "r") as f:
    for line in f:
        if line.startswith("%"):
            header_lines.append(line.rstrip("\n"))
        elif line.strip():
            data_lines.append(line.rstrip("\n"))

# ---- modify header ----
new_header = []
for h in header_lines:
    if "latitude(deg)" in h:
        h = (
            h.rstrip()
            + "   vn(m/s)   ve(m/s)   vu(m/s)"
            + "   sdvn(m/s)   sdve(m/s)   sdvu(m/s)"
        )
    new_header.append(h)

# ---- modify data ----
new_data = []
for line in data_lines:
    new_line = (
        line
        + "   0.0000   0.0000   0.0000"
        + "   30.0000   30.0000   30.0000"
    )
    new_data.append(new_line)

# ---- write output ----
with open(out_file, "w") as f:
    for h in new_header:
        f.write(h + "\n")
    for d in new_data:
        f.write(d + "\n")

print(f"Written: {out_file}")