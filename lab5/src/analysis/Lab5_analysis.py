import numpy as np
from scipy.integrate import cumtrapz
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import pandas as pd


def distortion_model(mag_field_x, mag_field_y, calibrated_params):
    scaling_factor = max(np.max(np.abs(mag_field_x)), np.max(np.abs(mag_field_y))) / max(np.abs(calibrated_params[0]), np.abs(calibrated_params[1]))
    # Calculate calibrated magnetometer data
    x = (mag_field_x - calibrated_params[0]) * scaling_factor
    y = (mag_field_y - calibrated_params[1]) * scaling_factor
    return np.array([x, y])


def circle_model(params, x, y):
    # Unpack parameters
    m_x, m_y, r = params
    # Compute the model values
    return (x - m_x)**2 + (y - m_y)**2 - r**2


def residual(params, x, y):
    return circle_model(params, x, y)


# Read data from CSV file
df = pd.read_csv('/home/rohit/EECE5554/Lab_4,5/circlevectornav.csv')

mag_field_x = df.iloc[:, 6].values
mag_field_y = df.iloc[:, 7].values


# Initial guess for parameters
p0 = [np.mean(mag_field_x), np.mean(mag_field_y), np.std(mag_field_x + mag_field_y) / 2]

lsq_min = least_squares(residual, p0, args=(mag_field_x, mag_field_y))

calibrated_data = distortion_model(mag_field_x, mag_field_y, lsq_min.x)


# Calculate center and radius of the ideal circle
center_x = np.mean(calibrated_data[0])
center_y = np.mean(calibrated_data[1])
radius = np.sqrt(np.mean((calibrated_data[0] - center_x)**2 + (calibrated_data[1] - center_y)**2))

# Generate theta values for the circle
theta = np.linspace(0, 2*np.pi, 100)
circle_x = center_x + radius * np.cos(theta)
circle_y = center_y + radius * np.sin(theta)


plt.figure(figsize=(12, 6))
plt.suptitle('Magnetometer circle data before and after the correction')

# Original data
plt.subplot(1, 2, 1)
plt.scatter(mag_field_x, mag_field_y, label='Original Data', color='green')

# plt.scatter(0, 0, color='black', label='Origin')
plt.plot(circle_x, circle_y, color='blue', linestyle='--', label='Ideal Circle')
plt.xlabel('X Component of Magnetic Field [Gauss]')
plt.ylabel('Y Component of Magnetic Field [Gauss]')
plt.title('Original Magnetometer Data')
plt.legend()
plt.grid()

# Calibrated data
plt.subplot(1, 2, 2)
plt.scatter(calibrated_data[0], calibrated_data[1], label='Calibrated Data', color='orange')
# plt.scatter(0, 0, color='black', label='Origin')
plt.plot(circle_x, circle_y, color='blue', linestyle='--', label='Ideal Circle')
plt.xlabel('X Component of Magnetic Field [Gauss]')
plt.ylabel('Y Component of Magnetic Field [Gauss]')
plt.title('Calibrated Magnetometer Data')
plt.grid()
plt.legend()

#plt.tight_layout()
plt.subplots_adjust(top=0.85)
# plt.show()

print("Calibration parameters:", lsq_min.x)














import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy.integrate as integrate

# Function to calculate yaw angle from magnetometer data
def calculate_yaw_angle(magnetometer_x, magnetometer_y):
    return np.arctan2(magnetometer_x, magnetometer_y)

# Function to apply distortion model (soft iron calibration)
def distortion_model(mag_field_x, mag_field_y, calibrated_params):
    scaling_factor = max(np.max(np.abs(mag_field_x)), np.max(np.abs(mag_field_y))) / max(np.abs(calibrated_params[0]), np.abs(calibrated_params[1]))
    # Calculate calibrated magnetometer data
    x = (mag_field_x - calibrated_params[0]) * scaling_factor
    y = (mag_field_y - calibrated_params[1]) * scaling_factor
    return np.array([x, y])

# Read data from CSV file
df = pd.read_csv('Lab_4,5/BostonIMU.csv')

mag_field_x = df.iloc[:, 6].values
mag_field_y = df.iloc[:, 7].values

# Apply distortion model (soft iron calibration)
calibrated_data = distortion_model(mag_field_x, mag_field_y, lsq_min.x)
time = df['Seconds'].astype(float) + df['NanoSeconds'].astype(float) / 1e9
# Calculate yaw angles
raw_yaw_angles = calculate_yaw_angle(mag_field_x, mag_field_y)
corrected_yaw_angles = calculate_yaw_angle(calibrated_data[0], calibrated_data[1])

# Convert time to a NumPy array
time = time.to_numpy()

plt.figure(figsize=(12, 8))
plt.title('Raw vs. Corrected Yaw Angle before and after hard and soft iron calibration')
# Raw yaw angle

plt.plot(time, raw_yaw_angles, label='Raw Yaw Angle')
plt.xlabel('Time(s)')
plt.ylabel('Yaw Angle (radians)')
# Corrected yaw angle

plt.plot(time, corrected_yaw_angles, label='Corrected Yaw Angle', color='orange')
plt.xlabel('Time(s)')
plt.ylabel('Yaw Angle (radians)')
plt.legend()
plt.grid()




























import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

# Function to wrap angle values between -π and π
def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

# Assuming gyro data is stored in DataFrame df
gyro_z = df['ang vel z'].values

# Integrate yaw rate from gyro to get yaw angle
rotation_z = cumulative_trapezoid(gyro_z, time, initial=0)

# Wrap angle values between -π and π
rotation_z_wrapped = wrap_angle(rotation_z)

# Plot corrected magnetometer yaw angle and wrapped gyro yaw angle vs. time
plt.figure(figsize=(12, 6))
plt.plot(time, corrected_yaw_angles, label='Corrected Magnetometer Yaw Angle', color='orange')
plt.plot(time, rotation_z_wrapped, label='Wrapped Gyro Yaw Angle', color='green')
plt.xlabel('Time(s)')
plt.ylabel('Yaw Angle (radians)')
plt.title('Comparison of Magnetometer and Gyroscope Yaw Angle')
plt.legend()
plt.grid()
# plt.show()





import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Function to wrap angle values between -π and π
def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

# Function to design a Butterworth filter
def butter_lowpass_filter(data, cutoff_freq, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def butter_highpass_filter(data, cutoff_freq, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    y = filtfilt(b, a, data)
    return y

# Define filter parameters
fs = 40  # Sample frequency (Hz)
cutoff_low = 0.1  # Low-pass filter cutoff frequency (Hz)
cutoff_high = 0.5  # High-pass filter cutoff frequency (Hz)
order =4  # Filter order

# Initialize variables
filtered_yaw_mag = butter_lowpass_filter(corrected_yaw_angles, cutoff_low, fs, order)
filtered_yaw_gyro = butter_highpass_filter(rotation_z, cutoff_high, fs, order)
filtered_yaw_complementary = np.zeros_like(time)
prev_filtered_yaw = 0.0

# Process data with complementary filter
for i in range(1, len(time)):
    # Combine filtered magnetometer and gyro estimates using complementary filter
    alpha = 0.5  # Weight for magnetometer estimate
    filtered_yaw_complementary[i] = alpha * (prev_filtered_yaw + filtered_yaw_gyro[i] / fs) + (1 - alpha) * filtered_yaw_mag[i]
    
    # Wrap angle values between -π and π
    filtered_yaw_complementary[i] = wrap_angle(filtered_yaw_complementary[i])
    filtered_yaw_mag[i]=wrap_angle(filtered_yaw_mag[i])
    filtered_yaw_gyro[i]=wrap_angle(filtered_yaw_gyro[i])
    
    # Update previous filtered yaw
    prev_filtered_yaw = filtered_yaw_complementary[i]

# Plot filtered yaw angle
plt.figure(figsize=(10, 8))
plt.suptitle('Filter output Yaw Angle')
# Low-pass filter of magnetometer data
plt.subplot(2, 2, 1)
plt.plot(time, filtered_yaw_mag, label='Low-pass Filter (Magnetometer Data)', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (radians)')
plt.title('Low-pass Filter (Magnetometer Data)')
plt.legend()
plt.grid()

# High-pass filter of gyro data
plt.subplot(2, 2, 2)
plt.plot(time, filtered_yaw_gyro, label='High-pass Filter (Gyro Data)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (radians)')
plt.title('High-pass Filter (Gyro Data)')
plt.legend()
plt.grid()

# Complementary filter output
plt.subplot(2, 2, 3)
plt.plot(time, filtered_yaw_complementary, label='Complementary Filter Output', color='orange')
# plt.plot(time, rotation_z_wrapped, label='Wrapped Gyro Yaw Angle', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (radians)')
plt.title('Complementary Filter Output')
plt.legend()
plt.grid()

# IMU heading estimate
plt.subplot(2, 2, 4)
# plt.plot(time, raw_yaw_angles, label='IMU Heading Estimate', color='purple')
plt.plot(time, rotation_z_wrapped, label='Wrapped Gyro Yaw Angle', color='green')
plt.plot(time, filtered_yaw_complementary, label='Complementary Filter Output', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (radians)')
plt.title('IMU Heading Estimate')
plt.legend()
plt.grid()

plt.subplots_adjust(top=0.9, hspace=0.5) 

# plt.show()

plt.figure(figsize=(12,6))
plt.plot(time, rotation_z_wrapped, label='Wrapped Gyro Yaw Angle', color='green')
plt.plot(time, corrected_yaw_angles, label='Corrected Magnetometer Yaw Angle', color='blue')
plt.plot(time, filtered_yaw_complementary, label='Complementary Filter Output', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (radians)')
plt.title('IMU Heading Estimate')
plt.legend()
plt.grid()

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load IMU data from CSV


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load IMU data from CSV


# Extract relevant columns

imu_acc = df['lin acc x']

x = np.mean(imu_acc)
linear_acc = imu_acc - x

# Integrate forward acceleration to estimate forward velocity
forward_velocity_adjusted = cumulative_trapezoid(linear_acc, time, initial=0) 

# Set negative velocities to zero
forward_velocity_adjusted[forward_velocity_adjusted < 0] = 0

# Integrate raw forward acceleration to obtain raw forward velocity
forward_velocity_raw = cumulative_trapezoid(imu_acc, time, initial=0)

# Plot the adjusted forward velocity and raw forward velocity
plt.figure(figsize=(12, 6))
plt.plot(time, forward_velocity_adjusted, label='Forward Velocity Adjusted')
plt.plot(time, forward_velocity_raw, label='Forward Velocity Raw')
plt.xlabel('Time (s)')
plt.ylabel('Forward Velocity (m/s)')
plt.title('Forward Velocity Estimation from IMU before and after any adjustments')
plt.legend()
plt.grid(True)
# plt.show()





import numpy as np
import pandas as pd

# Load GPS data from CSV

gps_data = pd.read_csv('Lab_4,5/Bostongpscsv.csv')

# Extract relevant columns
gps_time = gps_data['Seconds'].astype(float) + gps_data['NanoSeconds'].astype(float) * 1e-9  # Combine Seconds and NanoSeconds to get time in seconds
gps_time=np.array(gps_time)

# Extract relevant columns
UTM_easting = gps_data['UTM_Easting']
UTM_northing = gps_data['UTM_Northing']

# Calculate distance between consecutive points
distance = np.sqrt((np.diff(UTM_northing))**2 + (np.diff(UTM_easting))**2)

# Calculate velocity from distance and time differences
gps_vel = distance / np.diff(gps_time)

# Plot velocity estimate from GPS
plt.figure(figsize=(12,6))
plt.plot(gps_time[1:], gps_vel, label='GPS Velocity')
plt.plot(time, forward_velocity_adjusted, label='Forward Velocity Adjusted')
plt.xlabel('Time(s)')
plt.ylabel('Velocity')
plt.title('Velocity Estimate from GPS')
plt.legend()
plt.grid(True)







import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



# Calculate distance between consecutive GPS points
distance = np.sqrt((np.diff(UTM_northing))**2 + (np.diff(UTM_easting))**2)

# Integrate forward velocity to obtain displacement for IMU data

imu_displacement = cumulative_trapezoid(forward_velocity_adjusted, time, initial=0)
# Integrate distance to obtain displacement for GPS data

gps_displacement=cumulative_trapezoid(distance, gps_time[1:], initial=0)
# Plot IMU and GPS calculated displacements
plt.figure(figsize=(12, 6))
plt.plot(time, imu_displacement, linewidth=2, label='IMU Calculated Displacement')
plt.plot(gps_time[1:], gps_displacement, linewidth=2, label='GPS Calculated Displacement')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (m)')
plt.title('Displacement from IMU and GPS before adjustment')
plt.legend()
plt.grid(True)


















import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load IMU data from CSV
y_observed = df['lin acc y'].values

x1dot=forward_velocity_adjusted
gyro_z = df['ang vel z'].values

# Calculate y2dot
y2dot = gyro_z * x1dot

# Plot Y_observed vs wX(dot)





y_observed_filtered = butter_lowpass_filter(y_observed, cutoff_low, fs, order)
y2dot_filtered = butter_lowpass_filter(y2dot, cutoff_low, fs, order)

# Plot Y_observed after low-pass filter vs wX(dot)
plt.figure(figsize=(12, 6))
plt.suptitle('Y observed vs wx\' Before and After Low-Pass filter')



plt.subplot(2,1,1)
plt.plot(time, y_observed,  label='Y observed')
plt.plot(time, y2dot,  label='wX(dot)')
plt.legend()
plt.grid(True)
plt.title('Y observed vs wX\' before lpf')
plt.xlabel('Time(s)')
plt.ylabel('Acceleration')
plt.subplots_adjust(top=0.9, hspace=0.5) 






plt.subplot(2,1,2)
plt.plot(time, y_observed_filtered, linewidth=2, label='Y observed lpf')
plt.plot(time, y2dot_filtered, label='wX(dot) lpf')
plt.legend()
plt.grid(True)
plt.title('Y observed vs wX\' after lpf ')
plt.xlabel('Time(s)')
plt.ylabel('Acceleration')

# plt.show()















import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
from scipy.spatial.transform import Rotation as R

# Load IMU data from CSV
df = pd.read_csv('/home/rohit/EECE5554/Lab_4,5/BostonIMU.csv')

# Convert quaternion to Euler angles (roll, pitch, yaw)
# quat = df[['w', 'x', 'y', 'z']].to_numpy()
# r = R.from_quat(quat)
# RPY = r.as_euler('zyx')

# Extract relevant columns
intYaw = cumtrapz(df['ang vel z'],x=time)
yaw_z = intYaw
fv = np.unwrap(forward_velocity_adjusted)
fv=fv[1:]
time=time[1:]

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

yaw_z_unwrapped = wrap_to_pi(yaw_z)


rot = 67*(np.pi / 180)
# rot=0

# Calculate unit vectors

unit1 = np.sin(yaw_z_unwrapped - rot) * fv
unit2 = np.cos(yaw_z_unwrapped - rot) * fv


# Calculate ve and vn
ve = unit1 - unit2
vn = unit1 + unit2
                                                                        
# Integrate ve and vn to obtain xe and xn
xe = cumtrapz(ve, x=time)
xn = cumtrapz(vn, x=time)

xe = xe
xn = xn
xc = np.mean((y2dot - x1dot))



# # Normalize xe and xn
# xe = xe - np.mean(xe)
# xn = xn - np.mean(xn)



UTMeast = gps_data['UTM_Easting']
UTMnorth = gps_data['UTM_Northing']

# Calculate scale factors
scale1 = (max(UTMeast) - min(UTMeast)) / (max(xe) - min(xe))
scale2 = (max(UTMnorth) - min(UTMnorth)) / (max(xn) - min(xn))
print(scale1)
print(scale2)
# Rescale xe and xn
# xe = xe * scale1
# 0.652207214225221 
xe = xe *  0.4

xn = xn * 0.4

plt.figure(figsize=(12, 6))
# plt.subplot(1,2,1)
# plt.suptitle('Trajectory of Vehicle')
plt.plot(xe-np.mean(xe)+16, xn-np.mean(xn) +19 , linewidth=2, label='IMU Calculated Trajectory')


# plt.subplot(1,2,2)
plt.plot((UTMeast - np.mean(UTMeast)).to_numpy(), (UTMnorth - np.mean(UTMnorth)).to_numpy(), linewidth=2, label='GPS Calculated Trajectory', color="red")
# plt.title('GPS Easting vs Northing')
plt.title('GPS IMU Easting vs Northing')
plt.xlabel('Xe')
plt.ylabel('Xn')

plt.grid(True)
plt.legend()
# plt.show()
indices_greater_than_156 = [index for index, value in enumerate(xn-np.mean(xn) +19) if value > 121 and value < 122]
print(indices_greater_than_156)  # Output: [1, 3, 4]

print("Mean of (y2dot - x1dot):", xc)
print(time[29840]-time[16517])
