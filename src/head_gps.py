#!/usr/bin/python
'''
Publish GPS data
GPS must be port ttyACM0 at 115200 baud
* USB GPS transmits at 4800 baud so this should be changed if used

Author: Adam Stager (astager@udel.edu)
'''
import rospy
import serial,csv,time,math
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

def gps_driver(gps_data, serial_id, nmea_id):
    #Initialize GPS
    print('Streaming GPS data...')

    while not rospy.is_shutdown():
        line = serial_id.readline()
        #print(line)
        data = line.split(',')

        if data[0] ==  nmea_id: # $GPGGA for USB GPS, GNGGA for EMLID and Lecia
            nmea_time = data[1] #time from GPS
            lat = data[2]
            degWhole = int(math.floor(float(lat)/100))
            degDec = (float(lat)-degWhole*100)/60
            deg = degWhole + degDec
            lat = deg
            latDir = data[3]
            if latDir == 'S':
                lat = -1*lat
            lon = data[4]
            degWhole = int(math.floor(float(lon)/100))
            degDec = (float(lon)-degWhole*100)/60
            deg = degWhole + degDec
            lon = deg
            lonDir = data[5]
            if lonDir == 'W':
                lon = -1*lon
            satQuality = data[6]
            satNum = data[7]

            #NavData.altitude = round(alt,10)
	    gps_data.latitude = round(lat,10)
	    gps_data.longitude = round(lon,10)
	    gps_data.status.status = int(data[6])

            rospy.loginfo("Latitude: %s  Longitude: %s", round(lat,10), round(lon,10))
            head_pub.publish(gps_data)


if __name__ == "__main__":
    rospy.init_node('head_gps_driver')
    rate = rospy.Rate(5)  # Hz

    head_pub = rospy.Publisher('gps_head', NavSatFix,queue_size=1)

    # 4800 baud for USB GPS
    head = serial.Serial('/dev/ttyACM0', 115200, timeout = 2)  # 4800 baud for USB GPS

    gps_head = NavSatFix()
    gps_driver(gps_head, head, '$GNGGA')
