import numpy as np

gps_xs = np.genfromtxt("../config/log/Graph1.txt",delimiter=",",skip_header=True)
sigma = np.std(gps_xs[:,1])
x_bar = np.mean(gps_xs[:,1])
N = len(gps_xs)
print(f"GPS x: x_bar={x_bar},sigma={sigma},N={N}")

imu_axs = np.genfromtxt("../config/log/Graph2.txt",delimiter=",",skip_header=True)
sigma = np.std(imu_axs[:,1])
x_bar = np.mean(imu_axs[:,1])
N = len(imu_axs)
print(f"IMU ax: x_bar={x_bar},sigma={sigma},N={N}")
