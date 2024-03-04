import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

# Read the data from the CSV file
df = pd.read_csv("/home/rohit/gnss/walkingRTK.csv")

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
plt.title('RTK Scatter Plot of Northing vs. Easting with Line of Best Fit')

# Add legend
plt.legend()

plt.grid(True)
plt.show()


# Extract the northing and easting data
northing = df['UTM_Northing'].values.reshape(-1, 1)
easting = df['UTM_Easting'].values.reshape(-1, 1)

# Fit a line of best fit to the walking data using numpy's polyfit
slope, intercept = np.polyfit(easting.flatten(), northing.flatten(), 1)

# Calculate the predicted northing values based on the line of best fit
northing_predicted = slope * easting + intercept

# Calculate the error between the predicted and actual northing values
error = np.sqrt(mean_squared_error(northing, northing_predicted))


print("Root Mean Squared Error (RMSE) from Line of Best Fit to Walking Data:", error)

fq = df['Fix Quality'].to_numpy()
print("Fix Quality:", fq.mean())

