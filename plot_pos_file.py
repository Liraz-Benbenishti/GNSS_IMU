import pandas as pd
import matplotlib.pyplot as plt

# Load POS file
files = ["/app/gnss_imu_gili_first_headless_w_inter_vel.pos"]
# files = ["/app/gnss_imu_1934_sf.pos", "/app/gnss_imu_1934_sf_reset.pos"]
# file = "/app/gnss_imu_1934_sf.pos" # /app/gnss_imu_gili_first_headless_w_vel.pos"

for file in files:
    # Read, skip comments
    df = pd.read_csv(
        file,
        delim_whitespace=True,
        comment="%",
        header=None,
        usecols=[2, 3, 4],  # lat, lon, height
        names=["lat", "lon", "h"]
    )
    df.to_csv("/app/gnss_ex_imu_fusion_results.csv", index=False)

    # Plot trajectory

    # from simplekml import Kml

    # kml = Kml()

    # # Add line string (trajectory)
    # kml.newlinestring(name="/app/gnssex_imu_fusion_results.csv", coords=list(zip(df["lon"], df["lat"], df["h"])))

    # # Save KML
    # kml_file = "/app/gnss_example_path.kml"
    # kml.save(kml_file)

    # print("Saved KML:", kml_file)


    plt.figure(figsize=(8, 8))
    plt.plot(df["lon"], df["lat"], marker='.', linestyle='-', color='blue')
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("RTKLIB Position")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
