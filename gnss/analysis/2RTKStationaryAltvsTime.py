#Stationary Alt vs Time
import pandas as pd
import matplotlib.pyplot as plt
# Read the CSV file into a pandas DataFrame
df = pd.read_csv("/home/rohit/gnss/OpenRTK.csv")
#Occluded 
df1 = pd.read_csv("/home/rohit/gnss/OcculededRTK.csv")
df['Time']= df['Seconds'] #+ df['NanoSeconds']
df1['Time']= df1['Seconds']
print(df['Time'])
# Plot 
plt.figure(figsize=(100, 100))
plt.scatter(df['Time'],df['Altitude'],  color='blue', alpha=0.5, label="Stationary Open")
plt.scatter(df1['Time'],df1['Altitude'],  color='red', alpha=0.5, label="Stationary Occluded")
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Moving Altitude (m)')
plt.title('RTK - Plot of Stationary Altitude VS Time')
plt.grid(True)
plt.show()