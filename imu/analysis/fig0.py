import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data
data = pd.read_csv('/home/rohit/EECE5554/csv/testcsv.csv') 

# Convert rad/s to deg/s
data['x_deg_per_s'] = data['ang vel x'] * (180 / np.pi)
data['y_deg_per_s'] = data['ang vel y'] * (180 / np.pi)
data['z_deg_per_s'] = data['ang vel z'] * (180 / np.pi)
data['time'] = data['Seconds'] + data['NanoSeconds'] * 1e-9

# Set a consistent color cycle
colors = ['r', 'g', 'b']

# Create a figure and subplots with increased spacing
fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True, gridspec_kw={'hspace': 0.4})

# Plot each axis with a specified color and marker
for i, axis in enumerate(['X', 'Y', 'Z']):
    axs[i].plot(data['time'].values, data[f'{axis.lower()}_deg_per_s'].values, 
                color=colors[i], label=f'{axis} Rotational Rate')

    # Customize each subplot
    axs[i].set_ylabel(f'{axis} Rotational Rate (deg/s)')
    axs[i].legend()
    axs[i].grid(True)

# Set common labels and title
plt.xlabel('Time (s)')
plt.suptitle('Rotational Rate from Gyro vs Time', fontsize=16)  # Adjust font size here
plt.tight_layout()
plt.show()
