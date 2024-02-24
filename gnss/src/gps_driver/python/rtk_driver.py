#!/usr/bin/env python3
# license removed for brevity
import os
import utm
import time 
import serial
import rospy
import rosbag
from std_msgs.msg import Header
from gps_driver.msg import Customrtk



def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(serialPortAddr,rospy.get_param("~baud", "4800"))
    try:
        gnggaRead = serialPort.readline().decode('utf-8').strip()
        serialPort.close()
    except UnicodeDecodeError:
        gnggaRead= serialPort.readline().decode('utf-8').strip()
        serialPort.close()
    print(gnggaRead)
    # #Do not modify
    return gnggaRead


def degMinstoDegDec(Latorlong):
    deg = float(Latorlong)//100 #Replace 0 with a line of code that gets just the degrees from LatOrLong
    mins = float(Latorlong)%100 #Replace 0 with a line of code that gets just the minutes from LatOrLong
    degDec = float(mins/60) #Replace 0 with a line of code that converts minutes to decimal degrees
    #print(deg+degDec)
    return (deg+degDec)



def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W": #Replace the blank string with a value
        LatOrLong = -1*float(LatOrLong)
        #print("Longitude: "+ str(LatOrLong))
    if LatOrLongDir == "S": #Replace the blank string with a value
        LatOrLong = -1*float(LatOrLong)
        #print("Latitude: "+ str(LatOrLong))
    return float(LatOrLong)



def isgnggainString(inputString):
    if inputString.find("$GNGGA") == 0: #replace 1 == 1 with condition to be checked for inputString
        print('Great success!')
        #print(inputString)
        f=1
        return f

    else:
        print('gngga not found in string')
        f=0
        return f


def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0] #Again, replace these with values from UTMVals
    UTMNorthing = UTMVals[1] 
    UTMZone = UTMVals[2]
    UTMLetter = "'"+UTMVals[3]+"'"
    #print(UTMVals)
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]




def UTCtoUTCEpoch(UTC):
    UTCinSecs = (UTC // 10000)*3600 + ((UTC // 100) %  100) * 60 + (UTC % 100)
    TimeSinceEpoch = time.time() #Replace with a 1-line method to get time since epoch
    TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) #Use the time since epoch to get the time since epoch *at the beginning of the day*
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime) #Replace with a 1-line calculation to get total seconds as an integer
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * 1e9) #Replace with a 1-line calculation to get remaining nanoseconds as an integer (between CurrentTime and CurrentTimeSec )
    #print(CurrentTimeSec)
    #print(CurrentTimeNsec)
    return [CurrentTimeSec, CurrentTimeNsec]

    
if __name__ == "__main__":
    i=1
    rospy.init_node('rtk_gnss', anonymous=True)
    pub = rospy.Publisher('rtk_gnss', Customrtk, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Publisher Node Started, now publishing messages")
    bag=rosbag.Bag(os.getcwd()+'/testrtk.bag','w')

    #print('1')
    while not rospy.is_shutdown():
        serialPortAddr = rospy.get_param("~port", "/dev/pts/2") 
        gnggaRead = ReadFromSerial(serialPortAddr)
        #gnggaRead = '$GNGGA,165027.00,4220.2947440,N,07105.1903912,W,5,20,0.8,23.896,M,-28.725,M,1.0,0061*5E'
        # print('2')
        f1 = isgnggainString(gnggaRead)
        if f1 == 1:
                
             try:
                # print('3')
                gnggaSplit = gnggaRead.split(',') #Put code here that will split gnggaRead into its components. This should only take one line.

                size=len(gnggaSplit)
                if gnggaSplit[1] == '':
                    continue
                else:
                    UTC = float(gnggaSplit[1])
                
                if gnggaSplit[2] == '':
                    continue
                else:
                    Latitude = float(gnggaSplit[2])

                if gnggaSplit[3] == '':
                     continue
                else:
                    LatitudeDir = gnggaSplit[3]

                
                if gnggaSplit[4] == '':
                     continue
                else:
                    Longitude = float(gnggaSplit[4])
                
                if gnggaSplit[5] == '':
                     continue
                else:
                    LongitudeDir = gnggaSplit[5]
                
                if gnggaSplit[6] == '':
                     continue
                else:
                    fix_quality = int(gnggaSplit[6])
                
                if gnggaSplit[8] == '':
                    continue
                else:
                    HDOP = float(gnggaSplit[8])
                
                # print("4")    
                Latitudeg=degMinstoDegDec(Latitude)
                Longitudeg= degMinstoDegDec(Longitude)
                LatitudeSigned = LatLongSignConvetion(Latitudeg, LatitudeDir)
                LongitudeSigned = LatLongSignConvetion(Longitudeg, LongitudeDir)
                [EASTING, NORTHING, ZONE_NUMBER, ZONE_LETTER] =convertToUTM(LatitudeSigned, LongitudeSigned)
                CurrentTime = UTCtoUTCEpoch(UTC)
                # print('5')
                msg=Customrtk()
                msg.header = Header()
                msg.header.seq=i
                msg.header.frame_id = 'RTK1_Frame'
                msg.header.stamp.secs = CurrentTime[0]
                msg.header.stamp.nsecs = CurrentTime[1]
                msg.latitude = LatitudeSigned
                msg.longitude = LongitudeSigned
                msg.altitude = float(gnggaSplit[9])
                msg.utm_easting = EASTING
                msg.utm_northing = NORTHING
                msg.zone = ZONE_NUMBER
                msg.letter = ZONE_LETTER
                msg.fix_quality = fix_quality
                msg.hdop = HDOP
                msg.gngga_read = gnggaRead
                pub.publish(msg)
                bag.write('rtk_gnss',msg)
                rate.sleep()
                print("------------------------------------------")
                print('Reading:', gnggaRead)
                print("Sqeunce: ", i)
                print()
                print("UTC: " +str(UTC))
                print('Latitude: ', Latitude)
                print('LatitudeDir: ', LatitudeDir)
                print('Longitude: ', Longitude)
                print('LongitudeDir: ', LongitudeDir)
                print('Fix Quality: ', fix_quality)
                print('HDOP: ', HDOP)
                print()
                print('LatitudeSigned: ',LatitudeSigned)
                print('LongitudeSigned: ', LongitudeSigned)
                print('CurrentTimeSec: ', CurrentTime[0])
                print('CurrentTimeNsec:', CurrentTime[1])
                print()
                print('EASTING: ', EASTING)
                print('NORTHING: ', NORTHING)
                print('ZONE_NUMBER: ', ZONE_NUMBER)
                print('ZONE_LETTER: ', ZONE_LETTER)
                print("------------------------------------------")
                
                if i == 322:
                    print("stop")
                i=i+1
             except:
                  continue
    print("Saving .bag file")
    bag.close()


