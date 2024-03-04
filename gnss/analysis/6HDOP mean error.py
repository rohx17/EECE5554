import pandas as pd
import matplotlib.pyplot as plt

# Load data for open environment from CSV
open_data = pd.read_csv("/home/rohit/gnss/OpenRTK.csv")
#Occluded 
occluded_data = pd.read_csv("/home/rohit/gnss/OcculededRTK.csv")

# Calculate reference point for open environment (e.g., average coordinates)
open_reference_point = (open_data['UTM_Easting'].mean(), open_data['UTM_Northing'].mean())

# Calculate reference point for occluded environment (e.g., average coordinates)
occluded_reference_point = (occluded_data['UTM_Easting'].mean(), occluded_data['UTM_Northing'].mean())

# Calculate error estimates for open environment
open_errors = ((open_data['UTM_Easting'] - open_reference_point[0])**2 + 
               (open_data['UTM_Northing'] - open_reference_point[1])*2)*0.5

# Calculate error estimates for occluded environment
occluded_errors = ((occluded_data['UTM_Easting'] - occluded_reference_point[0])**2 + 
                   (occluded_data['UTM_Northing'] - occluded_reference_point[1])*2)*0.5

# Calculate summary statistics for HDOP
open_hdop_mean = open_data['HDOP'].mean()
open_hdop_median = open_data['HDOP'].median()
open_hdop_std = open_data['HDOP'].std()
open_hdop_max = open_data['HDOP'].max()

occluded_hdop_mean = occluded_data['HDOP'].mean()
occluded_hdop_median = occluded_data['HDOP'].median()
occluded_hdop_std = occluded_data['HDOP'].std()
occluded_hdop_max = occluded_data['HDOP'].max()

# Compare error estimates with HDOP
open_mean_error = open_errors.mean()
open_median_error = open_errors.median()
open_std_error = open_errors.std()
open_max_error = open_errors.max()

occluded_mean_error = occluded_errors.mean()
occluded_median_error = occluded_errors.median()
occluded_std_error = occluded_errors.std()
occluded_max_error = occluded_errors.max()

print("Comparison of Error Estimates and HDOP:")
print("Open Environment:")
print("Mean Error:", open_mean_error, "HDOP Mean:", open_hdop_mean)
print("Median Error:", open_median_error, "HDOP Median:", open_hdop_median)
print("Standard Deviation of Error:", open_std_error, "HDOP Std:", open_hdop_std)
print("Maximum Error:", open_max_error, "HDOP Max:", open_hdop_max)
print()
print("Occluded Environment:")
print("Mean Error:", occluded_mean_error, "HDOP Mean:", occluded_hdop_mean)
print("Median Error:", occluded_median_error, "HDOP Median:", occluded_hdop_median)
print("Standard Deviation of Error:", occluded_std_error, "HDOP Std:", occluded_hdop_std)
print("Maximum Error:", occluded_max_error, "HDOP Max:", occluded_hdop_max)

# Plotting
plt.figure(figsize=(10, 6))

# Plot error estimates for open environment
plt.scatter(open_data['HDOP'], open_errors, label='Open Environment', marker='o')

# Plot error estimates for occluded environment
plt.scatter(occluded_data['HDOP'], occluded_errors, label='Occluded Environment', marker='x')

# Add labels and title
plt.xlabel('HDOP')
plt.ylabel('Error Estimates (UTM)')
plt.title('Error Estimates and Comparison with HDOP')
plt.legend()

# Show plot
plt.grid(True)
plt.show()