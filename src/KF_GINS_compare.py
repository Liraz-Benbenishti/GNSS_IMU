# Demonstrate ability to translate from GNSS position to IMU position or vice-versa 
# using KF_GINS demo GNSS and truth data

import numpy as np
import matplotlib.pyplot as plt
from imu_transforms import Euler_to_CTM, LLH_to_ECEF, ECEF_to_NED
from imu_files import Read_GNSS_data
from imu_math import Lever_Arm
from os.path import join


# Input files
dataDir = r'..\data\KF_GINS'
truth_file = 'truth.nav'
gnss_in_file = 'gnss_ADIS16465_sf.pos'
sol_file = 'gnss_imu_ADIS16465_sf.pos'
wuhan_sol_file= 'NavResult_GNSSVEL.nav'

#r_lever_arm_b = np.array([-0.37, -0.008, 0.353])  # forward, right, down
r_lever_arm_b = np.array([-0.37, -0.008, 0.353])  # forward, right, down
title = 'ADIS16465'

def LLH_to_NED(pos_llh, origin_llh):
    pos_ecef = LLH_to_ECEF(pos_llh)
    origin_ecef = LLH_to_ECEF(origin_llh[None,:])[0]
    pos_ned, _ = ECEF_to_NED(pos_ecef, pos_ecef, origin_ecef)  # dummy vel
    return pos_ned

# Read and parse ground truth file and convert to NED
data = np.genfromtxt(join(dataDir, truth_file))
t_truth = data[:,1] % 3600  # strip hours and days
pos_truth_llh = data[:,2:5]
pos_truth_llh[:,:2] = np.deg2rad(pos_truth_llh[:,:2])
pos_truth_n = LLH_to_NED(pos_truth_llh, pos_truth_llh[0])
vel_truth_n = data[:,5:8]
rpy_truth_b_n = data[:,8:11] # body frame relative to NED

# Read and parse GNSS_IMU sol file and convert to NED
in_sol, num_sol, ok = Read_GNSS_data(join(dataDir, sol_file))
t_sol = in_sol[:,0] % 3600  # strip hours and days
pos_sol_llh = in_sol[:,1:4]
pos_sol_n = LLH_to_NED(pos_sol_llh, pos_truth_llh[0])
vel_sol_n = in_sol[:,14:17]
rpy_sol_b_n = in_sol[:,23:26]

# Read and parse GNSS input file and convert to NED
in_gnss, num_gnss, ok = Read_GNSS_data(join(dataDir, gnss_in_file))
t_gnss = in_gnss[:,0] % 3600  # strip hours and days
pos_gnss_llh = in_gnss[:,1:4]
pos_gnss_n = LLH_to_NED(pos_gnss_llh, pos_truth_llh[0])
vel_gnss_n = in_gnss[:,14:17]

# Read and parse Wuhan solution file and convert to NED
data = np.genfromtxt(join(dataDir, wuhan_sol_file))
t_wsol = data[:,1] % 3600  # strip hours and days
pos_wsol_llh = data[:,2:5]
pos_wsol_llh[:,:2] = np.deg2rad(pos_wsol_llh[:,:2])
pos_wsol_n = LLH_to_NED(pos_wsol_llh, pos_truth_llh[0])
vel_wsol_n = data[:,5:8]
rpy_wsol_b_n = data[:,8:11]

# Resample GNSS data to match truth data
pos_gnss_n_resamp, vel_gnss_n_resamp = np.zeros_like(pos_truth_n), np.zeros_like(vel_truth_n)
rpy_gnss_b_n_resamp = np.zeros_like(pos_truth_n)
for i in range(3):
    pos_gnss_n_resamp[:,i] = np.interp(t_truth, t_gnss, pos_gnss_n[:,i])
    vel_gnss_n_resamp[:,i] = np.interp(t_truth, t_gnss, vel_gnss_n[:,i])
    
# Resample KF_GINS solution data to match truth data
pos_wsol_n_resamp, vel_wsol_n_resamp = np.zeros_like(pos_truth_n), np.zeros_like(vel_truth_n)
rpy_wsol_b_n_resamp = np.zeros_like(pos_truth_n)
for i in range(3):
    pos_wsol_n_resamp[:,i] = np.interp(t_truth, t_wsol, pos_wsol_n[:,i])
    vel_wsol_n_resamp[:,i] = np.interp(t_truth, t_wsol, vel_wsol_n[:,i])
    rpy_wsol_b_n_resamp[:,i] = np.interp(t_truth, t_wsol, np.unwrap(rpy_wsol_b_n[:,i],period=360))
    
# Resample GNSS_IMU solution data to match truth data
pos_sol_n_resamp, vel_sol_n_resamp = np.zeros_like(pos_truth_n), np.zeros_like(vel_truth_n)
rpy_sol_b_n_resamp = np.zeros_like(pos_truth_n)
for i in range(3):
    pos_sol_n_resamp[:,i] = np.interp(t_truth, t_sol, pos_sol_n[:,i])
    vel_sol_n_resamp[:,i] = np.interp(t_truth, t_sol, vel_sol_n[:,i])
    rpy_sol_b_n_resamp[:,i] = np.interp(t_truth, t_sol, np.unwrap(rpy_sol_b_n[:,i],period=360))
    
# Resample truth data to match GNSS data
pos_truth_n_resamp, vel_truth_n_resamp = np.zeros_like(pos_gnss_n), np.zeros_like(vel_gnss_n)
rpy_truth_b_n_resamp = np.zeros_like(pos_gnss_n)
for i in range(3):
    pos_truth_n_resamp[:,i] = np.interp(t_gnss, t_truth, pos_truth_n[:,i])
    vel_truth_n_resamp[:,i] = np.interp(t_gnss, t_truth, vel_truth_n[:,i])
    rpy_truth_b_n_resamp[:,i] = np.interp(t_gnss, t_truth, np.unwrap(rpy_truth_b_n[:,i],period=360))
    
# Translate resampled truth data from IMU position to GNSS position
pos_truth_lev_n = np.zeros_like(pos_truth_n_resamp)
vel_truth_lev_n = np.zeros_like(vel_truth_n_resamp)
for i in range(len(pos_truth_n_resamp)):
    C_b_n = Euler_to_CTM(np.deg2rad(rpy_truth_b_n_resamp[i])).T   # body -> NED
    vel_truth_lev_n[i], pos_truth_lev_n[i] = Lever_Arm(C_b_n, 
        vel_truth_n_resamp[i], pos_truth_n_resamp[i], [0,0,0], r_lever_arm_b)

# plot positions
t_max = min(t_truth[-1],t_sol[-1])
n = np.where(t_truth<=t_max)[0][-1]
fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.suptitle('%s:Position errors (NED)' % title)
axes[0].plot(t_truth[:n] - t_sol[0], pos_truth_n[:n], '.-')
axes[0].plot(t_truth[:n] - t_sol[0], pos_sol_n_resamp[:n], '.-')
axes[0].plot(t_truth[:n] - t_sol[0], pos_wsol_n_resamp[:n], '.-')
axes[1].plot(t_gnss[:n] - t_sol[0], pos_truth_lev_n[:n], '.')
axes[1].plot(t_gnss[:n] - t_sol[0], pos_gnss_n[:n], '.')
axes[0].set_title('Positions')
axes[1].set_title('Positions')
axes[0].legend(['NT', 'ET', 'DT', 'NS', 'ES', 'DS', 'NW', 'EW', 'DW'])
axes[1].legend(['NTL', 'ETL', 'DTL', 'NG', 'EG', 'DG'])
for i in range(2):
    axes[i].grid()
    axes[i].set_xlabel('secs')
    axes[i].set_ylabel('meters')

# plot position errors
t_max = min(t_truth[-1],t_sol[-1])
n = np.where(t_truth<=t_max)[0][-1]
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('%s:Position errors (NED)' % title)
axes[0].plot(t_truth[:n] - t_sol[0], pos_sol_n_resamp[:n] - pos_truth_n[:n], '.-')
axes[1].plot(t_truth[:n] - t_sol[0], pos_wsol_n_resamp[:n] - pos_truth_n[:n], '.-')
axes[2].plot(t_gnss[:n] - t_sol[0], pos_gnss_n - pos_truth_lev_n, '.')
axes[0].set_title('GNSS_IMU solution errors')
axes[1].set_title('KF_GINS solution errors')
axes[2].set_title('GNSS solution errors')
for i in range(3):
    axes[i].legend(['dN', 'dE', 'dD'])
    axes[i].grid()
    axes[i].set_xlabel('secs')
    axes[i].set_ylabel('meters')

# plot velocity errors
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('%s:Velocity errors (NED)' % title)
axes[0].plot(t_truth[:n] - t_sol[0], vel_sol_n_resamp[:n] - vel_truth_n[:n], '.-') # GNSS_IMU is GNSS frame
axes[1].plot(t_truth[:n] - t_sol[0], vel_wsol_n_resamp[:n] - vel_truth_n[:n], '.-')  # Wuhan sol is IMU frame
axes[2].plot(t_gnss - t_sol[0], vel_gnss_n - vel_truth_lev_n, '.')
axes[0].set_title('GNSS_IMU solution errors')
axes[1].set_title('KF_GINS solution errors')
axes[2].set_title('GNSS solution errors')
for i in range(3):
    axes[i].legend(['dN', 'dE', 'dD'])
    axes[i].grid()
    axes[i].set_xlabel('secs')
    axes[i].set_ylabel('meters/sec')


# plot orientation errors
fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.suptitle('%s:Orientation errors (NED)' % title)
axes[0].plot(t_truth[:n] - t_sol[0], np.unwrap(rpy_sol_b_n_resamp[:n] - rpy_truth_b_n[:n], period=360))
axes[1].plot(t_truth[:n] - t_sol[0], np.unwrap(rpy_wsol_b_n_resamp[:n] - rpy_truth_b_n[:n], period=360))
axes[0].set_title('GNSS_IMU solution errors')
axes[1].set_title('KF_GINS solution errors')
for i in range(2):
    axes[i].legend(['dR', 'dP', 'dY'])
    axes[i].grid()
    axes[i].set_xlabel('secs')
    axes[i].set_ylabel('degrees')



