import pandas as pd
import numpy as np

filename = 'logfile_eh.csv'  # Replace with the path to your log file
data = pd.read_csv(filename)


time = data['time']
error = data['error']




# Compute mean and standard deviation
mean = np.mean(error)
std_dev = np.std(error)

print(f"Mean: {mean}")
print(f"Standard Deviation: {std_dev}")


