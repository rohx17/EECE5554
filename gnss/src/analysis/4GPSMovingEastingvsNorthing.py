#Stationary both plot together
import pandas as pd
import matplotlib.pyplot as plt

import numpy as np

# Load your CSV file
df = pd.read_csv("/home/rohit/gnss/WalkingGPS.csv")

# Extract the northing and easting data
northing = df['UTM_Northing'].to_numpy()
easting = df['UTM_Easting'].to_numpy()

# Calculate the line of best fit using numpy
slope, intercept = np.polyfit(easting, northing, 1)

# Create scatter plot
plt.figure(figsize=(8, 6))
plt.scatter(easting, northing, color='green', label='Data')
plt.plot(easting, slope * easting + intercept, color='red', label='Line of Best Fit')

# Add labels and title
plt.xlabel('Easting')
plt.ylabel('Northing')
plt.title('Scatter Plot of Northing vs. Easting with Line of Best Fit')

# Add legend
plt.legend()

plt.grid(True)
plt.show()

# Calculate the error from the line of best fit to the data
northing_predicted = slope * easting + intercept
error = np.linalg.norm(northing - northing_predicted) / len(northing)

print("Error from the line of best fit to the data:", error)
