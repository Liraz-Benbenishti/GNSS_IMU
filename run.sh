# Build the Docker image
docker build -t gnss-imu-fusion .

# Remove any existing container with the same name
docker rm -f gnss-solver-container
docker kill gnss-solver-container || true

# Necessary for GUI applications to run inside Docker
# or else, exception regarding "headless" mode will be thrown.
xhost +local:docker

# Run the Docker container and execute the GNSS logger parser
docker run -dit -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
--name gnss-solver-container -v /home/liraz/Downloads/gili2:/app gnss-imu-fusion bash
docker exec gnss-solver-container bash -c "python3 /code/plot_pos_file.py" # src/GNSS_IMU.py" # plot_pos_file.py" # fill_empty_pos_columns.py"
##gnss_logger_parser.py \
##--input_file /app/gnss_log_2026_01_15_16_51_23.txt"