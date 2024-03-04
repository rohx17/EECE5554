import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data
data = pd.read_csv('/home/rohit/EECE5554/csv/testcsv.csv') 
data['time'] = data['Seconds'] + data['NanoSeconds'] * 1e-9

# Set a consistent color cycle
colors = ['b', 'r', 'g']

# Create a figure and subplots with increased spacing
fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True, gridspec_kw={'hspace': 0.4})

# Plot each axis with a specified color and marker
for i, axis in enumerate(['X', 'Y', 'Z']):
    axs[i].plot(data['time'].values, data[f'lin acc {axis.lower()}'].values, 
                color=colors[i], label=f'{axis} Acceleration')

    # Customize each subplot
    axs[i].set_ylabel(f'{axis} Acceleration (m/s^2)')
    axs[i].legend()
    axs[i].grid(True)  # Add grid to each subplot

# Set common labels and title
plt.xlabel('Time (s)')
plt.suptitle('Acceleration from the accelerometer vs Time', fontsize=16)  # Adjust font size here
plt.tight_layout()
plt.show()
