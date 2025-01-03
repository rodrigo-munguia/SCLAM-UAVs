import matplotlib.pyplot as plt
import pandas as pd

# Read the log file into a pandas DataFrame
filename = 'logfile_lc.csv'  # Replace with the path to your log file
data = pd.read_csv(filename)


# Define the time range for plotting
start_time = 60.00  # Replace with the start time of your desired range
end_time = 121.30    # Replace with the end time of your desired range
# Filter the data based on the time range
filtered_dataX = data[(data['time'] >= start_time) & (data['time'] <= end_time)]
# Extract the columns for plotting
timeX = filtered_dataX['time']
velX_d = filtered_dataX['velX_d']
velX = filtered_dataX['velX']

start_time = 70.00  # Replace with the start time of your desired range
end_time = 121.30    # Replace with the end time of your desired range
# Filter the data based on the time range
filtered_dataY = data[(data['time'] >= start_time) & (data['time'] <= end_time)]
# Extract the columns for plotting
timeY = filtered_dataY['time']
velY_d = filtered_dataY['velY_d']
velY = filtered_dataY['velY']

start_time = 18.00  # Replace with the start time of your desired range
end_time = 60.30    # Replace with the end time of your desired range
# Filter the data based on the time range
filtered_dataZ = data[(data['time'] >= start_time) & (data['time'] <= end_time)]
# Extract the columns for plotting
timeZ = filtered_dataZ['time']
Z_d = filtered_dataZ['z_d']
Z = filtered_dataZ['z']

start_time = 18.00  # Replace with the start time of your desired range
end_time = 80.30    # Replace with the end time of your desired range
# Filter the data based on the time range
filtered_dataYaw = data[(data['time'] >= start_time) & (data['time'] <= end_time)]
# Extract the columns for plotting
timeYaw  = filtered_dataYaw ['time']
Yaw_d = filtered_dataYaw['velYaw_d']
Yaw  = filtered_dataYaw['velYaw']


# -------------------------------------------------------------------------
#plt.figure(figsize=(10, 6))
fig1 = plt.figure(figsize=(8, 4))
# Plot time vs roll_d and time vs roll on the same plot
plt.plot(timeX, velX_d, linestyle='--', color='b', label='velX_d')
plt.plot(timeX, velX, linestyle='-', color='r', label='velX')
# Set labels and title
plt.xlabel('time')
plt.ylabel('Value')
plt.title('Time vs velX_d and velX')
plt.legend()
# Improve layout
plt.tight_layout()

# -------------------------------------------------------------------------
fig2 = plt.figure(figsize=(8, 4))
# Plot time vs roll_d and time vs roll on the same plot
plt.plot(timeY, velY_d, linestyle='--', color='b', label='velY_d')
plt.plot(timeY, velY, linestyle='-', color='r', label='velY')
# Set labels and title
plt.xlabel('time')
plt.ylabel('Value')
plt.title('Time vs velY_d and velY')
plt.legend()
# Improve layout
plt.tight_layout()

# -------------------------------------------------------------------------
fig3 = plt.figure(figsize=(8, 4))
# Plot time vs roll_d and time vs roll on the same plot
plt.plot(timeZ, Z_d, linestyle='--', color='b', label='Z_d')
plt.plot(timeZ, Z, linestyle='-', color='r', label='Z')
# Set labels and title
plt.xlabel('time')
plt.ylabel('Value')
plt.title('Time vs Z_d and Z')
plt.legend()
# Improve layout
plt.tight_layout()


# -------------------------------------------------------------------------
fig4 = plt.figure(figsize=(8, 4))
# Plot time vs roll_d and time vs roll on the same plot
plt.plot(timeYaw, Yaw_d, linestyle='--', color='b', label='velYaw_d')
plt.plot(timeYaw, Yaw, linestyle='-', color='r', label='velYaw')
# Set labels and title
plt.xlabel('time')
plt.ylabel('Value')
plt.title('Time vs velYaw_d and velYaw')
plt.legend()
# Improve layout
plt.tight_layout()







# Show the plot
plt.show()