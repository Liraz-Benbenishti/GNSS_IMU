from simplekml import Kml

kml = Kml()

# Add line string (trajectory)
kml.newlinestring(name="/app/gnss_imu_fusion_results.csv", coords=list(zip(df["lon"], df["lat"], df["h"])))

# Save KML
kml_file = "gnss_path.kml"
kml.save(kml_file)

print("Saved KML:", kml_file)
