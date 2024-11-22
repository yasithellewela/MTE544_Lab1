import pandas as pd
import numpy as np

# Load the CSV file
# change name based on file you want to run, q=0.3 & r=-0.5, q=0.7 & r=0.5, q=0.5 & r=0.3, q=0.5 & r=0.7 is copy 2,3,4,5 respectively
# point controller is copy 6
file_path = 'robotPose copy 6.csv'
data = pd.read_csv(file_path)
data.columns = data.columns.str.strip()

# Get EKF and odom values
kf_x = data['kf_x']
kf_y = data['kf_y']
odom_x = data['odom_x']
odom_y = data['odom_y']

# Compute RMSE for x and y
rmse_x = np.sqrt(np.mean((kf_x - odom_x) ** 2))
print(f"RMSE_X: {rmse_x}")
rmse_y = np.sqrt(np.mean((kf_y - odom_y) ** 2))

# Combine RMSE for 2D distance
rmse = np.sqrt(rmse_x**2 + rmse_y**2)

# Compute range of odometry values for x and y
range_odom_x = odom_x.max() - odom_x.min()
print(f"range_x: {range_odom_x}")
range_odom_y = odom_y.max() - odom_y.min()
print(f"range_y: {range_odom_y}")

# Combine range of odometry for 2D distance
range_odom = np.sqrt(range_odom_x**2 + range_odom_y**2)

# Compute relative error
relative_error = (rmse / range_odom) * 100

print(f"RMSE: {rmse}")
print(f"Range of odometry: {range_odom}")
print(f"Relative Error: {relative_error:.2f}%")
