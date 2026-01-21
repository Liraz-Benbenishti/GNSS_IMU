#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
 Phone-based GNSS/IMU Configuration (Dashboard-mounted smartphone)

 Device:
     DOOGEE V Max (Android phone)
     IMU data source: GNSS Logger
     Sensors used:
         - UncalibratedAccelerometer
         - UncalibratedGyroscope

 Mounting:
     - Phone placed on passenger-side dashboard
     - Exact location unknown
     - Orientation roughly fixed but may slip slightly due to friction
     - No rigid lever-arm calibration possible

 Intended use:
     - Vehicle navigation
     - Loosely-coupled GNSS/INS Kalman filter
     - Consumer-grade MEMS IMU assumptions

===============================================================================
"""

from imu_math import Init
import numpy as np

# =============================================================================
# IMU MODEL SELECTION
# =============================================================================

# 'PSD'      : continuous-time power spectral density model (recommended)
# 'random_walk' : discrete random walk model (less physical)
#
# For phone IMUs, PSD is preferred because:
# - Android sampling is irregular
# - Sensor noise is continuous-time in nature
imu_config = 'PSD'

# IMU sample rate in Hz.
# GNSS Logger typically outputs:
#   - 100 Hz or 200 Hz depending on device
#
# Valid range: 50 – 400 Hz
imu_sample_rate = 200


# =============================================================================
# IMU LEVER ARM (OFFSET FROM VEHICLE ORIGIN)
# =============================================================================

# Offset from vehicle reference point (usually rear axle center or GNSS antenna)
# Coordinate frame: [forward, right, down] in meters
#
# IMPORTANT:
# - Exact phone position is UNKNOWN
# - Phone may move slightly due to braking / turning
#
# Therefore:
#   Using an incorrect lever arm is WORSE than using none.
#   We intentionally set this to zero.
#
# Valid range if known: ±0.0 – 2.0 m
imu_offset = [0.0, 0.0, 0.0]


# =============================================================================
# IMU NOISE CHARACTERISTICS (PHONE-GRADE MEMS)
# =============================================================================

# ----------------------------
# White noise (measurement noise)
# ----------------------------

# Gyroscope noise PSD
# Units: deg / sec / sqrt(Hz)
#
# Typical values:
#   High-end automotive IMU: 0.001 – 0.005
#   Smartphone IMU:         0.01  – 0.05
#
gyro_noise_PSD = 0.02

# Accelerometer noise PSD
# Units: micro-g / sqrt(Hz)
#
# Typical values:
#   Automotive: 10 – 50
#   Smartphone: 150 – 300
#
accel_noise_PSD = 200


# ----------------------------
# Bias instability (random walk of bias)
# ----------------------------

# Accelerometer bias PSD
# Units: micro-g / sqrt(Hz)
#
# Phones suffer from:
# - Temperature drift
# - Mechanical stress
# - SoC heat coupling
#
# Valid range: 30 – 100
accel_bias_PSD = 50

# Gyroscope bias PSD
# Units: deg / sec / sqrt(Hz)
#
# Valid range for phones: 0.001 – 0.01
gyro_bias_PSD = 0.002


# ----------------------------
# Scale factor instability
# ----------------------------

# Standard deviation of accelerometer scale factor noise (dimensionless)
#
# Phones have noticeable scale drift over time.
# Valid range: 0.005 – 0.05
accel_scale_noise_SD = 0.01

# Standard deviation of gyroscope scale factor noise
#
# Valid range: 0.0005 – 0.01
gyro_scale_noise_SD = 0.001


# =============================================================================
# IMU ORIENTATION / MISALIGNMENT
# =============================================================================

# IMU axes relative to vehicle body frame (roll, pitch, yaw in degrees)
#
# We assume:
# - Phone is approximately flat on dashboard
# - Forward roughly aligned with vehicle
# - No precise calibration
#
# We DO NOT inject fine misalignment values.
# Any small misalignment will be absorbed by the filter.
#
# Valid range if roughly known: ±180 deg
imu_misalign = np.array([180.0, 0.0, 180.0])


# =============================================================================
# GNSS LEVER ARM
# =============================================================================

# GNSS antenna location relative to vehicle origin
#
# Using phone GNSS → antenna is inside the phone → unknown.
# Therefore we set to zero to avoid false corrections.
#
gnss_offset = [0.0, 0.0, 0.0]


# GNSS measurement outlier rejection threshold
# Max allowed 3D position sigma (meters)
#
# Valid range:
#   Urban: 5 – 20
#   Open sky: 3 – 10
#
gnss_pos_err_thresh = 10


# =============================================================================
# PROCESS NOISE INFLATION FACTORS
# =============================================================================

# Multipliers applied to nominal process noise to account for:
# - Android timestamp jitter
# - Sensor fusion in firmware
# - Unmodeled phone motion
#
# Order:
# [attitude, velocity, accel bias, gyro bias, accel scale, gyro scale]
#
# Values >1 intentionally reduce trust in IMU propagation.
#
# Typical phone range: 2 – 10
imu_noise_factors = [3, 3, 5, 5, 10, 10]

# GNSS measurement noise inflation
# [position, velocity]
#
# Helps in urban canyon / multipath
gnss_noise_factors = [1.5, 1.5]


# =============================================================================
# INITIAL STATE UNCERTAINTIES
# =============================================================================

init = Init()

# Attitude uncertainty (deg)
# Phones start poorly aligned.
# Valid range: 3 – 20 deg
init.att_unc = [5, 5, 15]

# Velocity uncertainty (m/s)
# Valid range: 0.1 – 1.0
init.vel_unc = [0.2, 0.2, 0.5]

# Position uncertainty (m)
# Valid range: 0.5 – 5.0
init.pos_unc = [1.0, 1.0, 2.0]

# Initial bias uncertainty
init.bias_acc_unc  = 0.3    # m/s^2   (0.2 – 1.0)
init.bias_gyro_unc = 0.3    # deg/s   (0.2 – 1.0)

# Initial scale uncertainty
init.scale_acc_unc  = 0.01
init.scale_gyro_unc = 0.01


# =============================================================================
# RUN PARAMETERS
# =============================================================================

# Estimate scale factor states online
# Strongly recommended for phone IMUs
scale_factors = True

# IMU timestamp correction (seconds)
#
# Android IMU timestamps often lag GNSS by 50–150 ms
# Negative value shifts IMU earlier.
#
# Valid range: -0.05 to -0.2
imu_t_off = -0.1

# Run direction
run_dir = [1]

# GNSS decimation
gnss_epoch_step = 1

# Output decimation
out_step = 1

# Output reference frame
out_frame = 'gnss'

# GNSS quality inflation
float_err_gain  = 2
single_err_gain = 5


# =============================================================================
# ZERO VELOCITY UPDATES (ZUPT / ZARU)
# =============================================================================

# Dashboard-mounted phone + vehicle
# → ZUPT is unreliable unless fully stopped.
zupt_enable = False

zupt_epoch_count = 30
zupt_accel_thresh = 0.4
zupt_gyro_thresh  = 0.6
zupt_vel_SD       = 0.05
zupt_accel_SD     = 0.05
zaru_gyro_SD      = 0.05


# =============================================================================
# INITIAL YAW ALIGNMENT
# =============================================================================

# Use GNSS velocity to initialize yaw
yaw_align = True

# Minimum speed for valid GNSS heading
# Valid range: 0.5 – 2.0 m/s
yaw_align_min_vel = 1.0

yaw_align_max_vel = 5.0

# Allow float solutions
yaw_align_min_fix_state = 2

# Magnetometer disabled (car environment is hostile)
init_yaw_with_mag = False


# =============================================================================
# NON-HOLONOMIC CONSTRAINTS (CAR MODEL)
# =============================================================================

# Enforce zero lateral and vertical velocity in body frame
# This is VERY powerful for stabilizing yaw with phone IMU.
nhc_enable = True

nhc_epoch_count = 15
nhc_min_vel = 3.0
nhc_gyro_thesh = 15
nhc_vel_SD = 0.2
nhc_vel_SD_coast = 0.1


# =============================================================================
# TESTING / DEBUG
# =============================================================================

disable_imu = False

start_coast = 40
end_coast   = 30
coast_len  = 15

num_epochs = 0

gyro_bias_err  = [0, 0, 0]
accel_bias_err = [0, 0, 0]

gyro_scale_factor  = [1, 1, 1]
accel_scale_factor = [1, 1, 1]

init_rpy = [0, 0, 0]


# =============================================================================
# PLOTTING
# =============================================================================

plot_results   = True
plot_bias_data = True
plot_imu_data  = False
plot_unc_data  = False
