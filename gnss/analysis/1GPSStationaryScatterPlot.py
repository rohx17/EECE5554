#Stationary both plot together
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file into a pandas DataFrame
df = pd.read_csv("/home/rohit/gnss/OpenGPS.csv")
#Occluded 
df1 = pd.read_csv("/home/rohit/gnss/OcculededGPS.csv")
# Display the first few rows of the DataFrame to understand its structure
print(df.head())
#Occluded 
print(df1.head())

# Extract the first point from each dataset
first_point = df.iloc[0]
#Occluded 
first_point1 =df1.iloc[0]

# Subtract the coordinates of the first point from all points in the dataset
df['dif_Easting'] = df['UTM_Easting'] - first_point['UTM_Easting']
df['dif_Northing'] = df['UTM_Northing'] - first_point['UTM_Northing']
#Occluded 
df1['dif_Easting'] = df1['UTM_Easting'] - first_point1['UTM_Easting']
df1['dif_Northing'] = df1['UTM_Northing'] - first_point1['UTM_Northing']
# Find the centroid of the adjusted dataset
centroid_easting = df['dif_Easting'].mean()
centroid_northing = df['dif_Northing'].mean()
#Occluded 
centroid_easting1 = df1['dif_Easting'].mean()
centroid_northing1 = df1['dif_Northing'].mean()


# Print the centroid and deviations
print("Centroid of adjusted dataset:")
print("Easting:", centroid_easting)
print("Northing:", centroid_northing)
#Occluded
print("Centroid of adjusted dataset (Occluded):")
print("Easting:", centroid_easting1)
print("Northing:", centroid_northing1)

# Print the first few rows of the DataFrame with adjusted columns
print(df.head())
print(df1.head())

# Subtract the centroid from each data point
df['dif_Easting'] -= centroid_easting
df['dif_Northing'] -= centroid_northing
#Occluded
df1['dif_Easting'] -= centroid_easting1
df1['dif_Northing'] -= centroid_northing1

# Plot the adjusted data
plt.figure(figsize=(8, 6))
plt.scatter(df['dif_Easting'], df['dif_Northing'], color='blue', alpha=0.5, label="Stationary Open")
#Occluded
plt.scatter(df1['dif_Easting'], df1['dif_Northing'], color='red', alpha=0.5, label="Stationary Occluded")
plt.xlabel('Difference in Easting (m)')
plt.ylabel('Difference in Northing (m)')
plt.title('ScatterPlots of Difference Easting vs. Northing (Stationary and Occluded)')
# Annotate the centroid values on the plot
plt.text(centroid_easting, centroid_northing, f'Centroid_Stationary: ({centroid_easting:.2f}, {centroid_northing:.2f})',
         fontsize=10, ha='right', va='top', color='blue')
plt.text(centroid_easting1, centroid_northing1, f'Centroid_Occluded: ({centroid_easting1:.2f}, {centroid_northing1:.2f})',
         fontsize=10, ha='center', va='center', color='red')
plt.legend()
plt.grid(True)
plt.show()



# Load your CSV files
df_open = df
df_occluded = df1

# Calculate the centroid of each data set
centroid_open = df_open[['UTM_Easting', 'UTM_Northing']].mean()
centroid_occluded = df_occluded[['UTM_Easting', 'UTM_Northing']].mean()

# Calculate the Euclidean distance from each point to the centroid for open data
errors_open = np.linalg.norm(df_open[['UTM_Easting', 'UTM_Northing']] - centroid_open, axis=1)

# Calculate the Euclidean distance from each point to the centroid for occluded data
errors_occluded = np.linalg.norm(df_occluded[['UTM_Easting', 'UTM_Northing']] - centroid_occluded, axis=1)

# Calculate the average error for both data sets
avg_error_open = np.mean(errors_open)
avg_error_occluded = np.mean(errors_occluded)

print("Average error from centroid for open data:", avg_error_open)
print("Average error from centroid for occluded data:", avg_error_occluded)
