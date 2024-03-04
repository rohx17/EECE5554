#!/usr/bin/env python3
import rospy
import rosbag
import serial
import numpy as np
import time
import os
from vn_driver.msg import Vectornav 



def ReadFromSerial(serialport):
    # serialport=serial.Serial(SerialPortAddr,rospy.get_param("~baud","115200"))
    vnymrRead=serialport.readline().decode('utf-8').strip()
    # print(vnymrRead)
    # serialport.close()
    return vnymrRead

def isVNYMRinString(inputString):
    if inputString.find("$VNYMR") == 0:
        f=1
        return f
    else:
        f=0
        return f


def convert_to_quaternion(roll, pitch, yaw):
    roll = roll*np.pi/180
    pitch = pitch*np.pi/180
    yaw = yaw*np.pi/180
    qx = (np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) - (np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
    qy = (np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
    qz = (np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)) - (np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
    qw = (np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)) + (np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
 
    return [qx, qy, qz, qw]


    
if __name__ == "__main__":
    
    rospy.init_node('imu', anonymous=True)
    pub = rospy.Publisher('/imu', Vectornav, queue_size=10)
    bag=rosbag.Bag(os.getcwd()+'/testimu.bag','w')
    rate = rospy.Rate(40)
    


    serialPortAddr = rospy.get_param("~port", "/dev/ttyUSB0") 
    serialport=serial.Serial(serialPortAddr,rospy.get_param("~baud","115200"))
    command = b'$VNWRG,07,40*XX\r\n'
    serialport.write(command)
    
    

    while not rospy.is_shutdown():
        vnymrRead = ReadFromSerial(serialport)
        now = rospy.Time.now()
        # vnymrRead = '$VNYMR,-165.981,-037.301,+001.245,+00.2901,+00.0720,+00.7482,-05.966,-00.150,-07.836,+00.001373,-00.001533,-00.001054*68'
        try:
            # print('2')
            f1 = isVNYMRinString(vnymrRead)
            
            if f1 == 1:
                # print('3')
                vnymrSplit = vnymrRead.split(',') 

                size=len(vnymrSplit)
    #"--------------------------------------------------------------------------------------------"                                
                if vnymrSplit[1] == '':
                    continue
                else:
                    yaw = float(vnymrSplit[1])
    #"--------------------------------------------------------------------------------------------"                
                if vnymrSplit[2] == '':
                    continue
                else:
                    pitch = float(vnymrSplit[2])
    #"--------------------------------------------------------------------------------------------"                                
                if vnymrSplit[3] == '':
                    continue
                else:
                    roll = float(vnymrSplit[3])
    #"--------------------------------------------------------------------------------------------"                                             
                if vnymrSplit[4] == '':
                    continue
                else:
                    xmag = float(vnymrSplit[4])
    #"--------------------------------------------------------------------------------------------"                
                if vnymrSplit[5] == '':
                    continue
                else:
                    ymag = float(vnymrSplit[5])
    #"--------------------------------------------------------------------------------------------"                                
                if vnymrSplit[6] == '':
                    continue
                else:
                    zmag = float(vnymrSplit[6])
    #"--------------------------------------------------------------------------------------------"                
                if vnymrSplit[7] == '':
                    continue
                else:
                    xacc = float(vnymrSplit[7])
    #"--------------------------------------------------------------------------------------------"                
                if vnymrSplit[8] == '':
                    continue
                else:
                    yacc = float(vnymrSplit[8])
    #"--------------------------------------------------------------------------------------------"                                
                if vnymrSplit[9] == '':
                    continue
                else:
                    zacc = float(vnymrSplit[9])
    #"--------------------------------------------------------------------------------------------"                
                if vnymrSplit[10] == '':
                    continue
                else:
                    xang = float(vnymrSplit[10])
    #"--------------------------------------------------------------------------------------------"                
                if vnymrSplit[11] == '':
                    continue
                else:
                    yang = float(vnymrSplit[11])
    #"--------------------------------------------------------------------------------------------"                                
                if vnymrSplit[12].split('*')[0] == '':
                    continue
                else:
                    zang = float(vnymrSplit[12].split('*')[0])
    #"--------------------------------------------------------------------------------------------"                            
                # print("4")    
                [qx, qy, qz, qw]=convert_to_quaternion(roll, pitch, yaw)

                # print('5')
                msg=Vectornav()
                # print('6')
                msg.header.frame_id = 'imu1_frame'
                msg.header.stamp.secs = now.secs
                msg.header.stamp.nsecs = now.nsecs
                # print('7')
                msg.mag_field.magnetic_field.x = xmag
                msg.mag_field.magnetic_field.y = ymag
                msg.mag_field.magnetic_field.z = zmag
                # print('8')
                msg.imu.orientation.x = qx
                msg.imu.orientation.y = qy
                msg.imu.orientation.z = qz
                msg.imu.orientation.w = qw
                
                msg.imu.linear_acceleration.x = xacc
                msg.imu.linear_acceleration.y = yacc
                msg.imu.linear_acceleration.z = zacc

                msg.imu.angular_velocity.x = xang
                msg.imu.angular_velocity.y = yang
                msg.imu.angular_velocity.z = zang
                msg.vnymr_read = vnymrRead
                
                # print('9')
                msg.vnymr_read = vnymrRead
                # rospy.loginfo(msg)
                pub.publish(msg)
                bag.write('imu',msg)
                # rate.sleep()
                print("------------------------------------------")
                print('Reading:', msg.vnymr_read)
                print()
                print('Frame ID: ', msg.header.frame_id)
                print('Current Time Sec: ', msg.header.stamp.secs)
                print('Current Time Nsec: ', msg.header.stamp.nsecs)

                print()
                print('Magnetometer Measurement in x-axis: %f Gauss' % msg.mag_field.magnetic_field.x)
                print('Magnetometer Measurement in y-axis: %f Gauss' % msg.mag_field.magnetic_field.y)
                print('Magnetometer Measurement in z-axis: %f Gauss' % msg.mag_field.magnetic_field.z)
                print()
                print('Quaternion Value W: %f scalar units' % msg.imu.orientation.w)
                print('Quaternion Value X: %f i' % msg.imu.orientation.x)
                print('Quaternion Value Y: %f j' % msg.imu.orientation.y)
                print('Quaternion Value Z: %f k' % msg.imu.orientation.z)
                print()
                print('Attitude yaw angle in degrees: %f deg' % yaw)
                print('Attitude pitch angle in degrees: %f deg' % pitch)
                print('Attitude roll angle in degrees: %f deg' % roll)
                print()
                print('Accelerometer Measurement in x-axis: %f m/s^2' % msg.imu.linear_acceleration.x)
                print('Accelerometer Measurement in Y-axis: %f m/s^2' % msg.imu.linear_acceleration.y)
                print('Accelerometer Measurement in z-axis: %f m/s^2' % msg.imu.linear_acceleration.z)
                print()
                print('Angular Rate in x-axis: %f rad/s' % msg.imu.angular_velocity.x)
                print('Angular Rate in y-axis: %f rad/s' % msg.imu.angular_velocity.y)
                print('Angular Rate in z-axis: %f rad/s' % msg.imu.angular_velocity.z)
                print()                
                print("------------------------------------------")
        except:
            continue
    print("------------------------------------------")
    print("Saving .bag file in " + os.getcwd())
    bag.close()

    fr=1
    while fr:
        # serialport=serial.Serial(serialPortAddr,rospy.get_param("~baud","115200"))
        serialport.write(b'$VNRRG,07*XX\r\n')
        response = serialport.readline().decode('utf-8').strip()
        if response.find("$VNRRG")==0:
            print(f"Response from register 07:", response)
            print("------------------------------------------")
            fr=0
            # serialport.close()
        else:
            continue
print("------------------------------------------")



