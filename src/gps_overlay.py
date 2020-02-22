#!/usr/bin/python
import serial,csv,time,math
#from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import math
import rospy
from sensor_msgs.msg import NavSatFix

'''class Map():
    def __init__(self, unit):
        self.name = unit

    def start_mapping(self):
        #Opens kml mapping file and writes so that it can be run in Google Earth
        global map_kml
        currentDate = time.strftime('%m-%d-%y__%H:%M:%S')
        map_kml = open("/home/adam/catkin_ws/src/gps_stable/src/data/mapping_%s.kml" %currentDate,"w")
        map_kml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>Robot Path</name><Style id=\"yellowLineGreenPoly\"><LineStyle><color>7f00ffff</color><width>4</width></LineStyle><PolyStyle><color>7f00ff00</color></PolyStyle></Style><Placemark><name>Robot Path</name><description>The path the robot takes</description><styleUrl>#yellowLineGreenPoly</styleUrl><LineString><extrude>0</extrude><tessellate>1</tessellate><altitudeMode>clampToGround</altitudeMode><coordinates>")
'''
def start_mapping():
    #Opens kml mapping file and writes so that it can be run in Google Earth
    global map_kml
    currentDate = time.strftime('%m-%d-%y__%H:%M:%S')
    map_kml = open("/home/seeterra/catkin_ws/src/gps_stable/src/data/mapping_%s.kml" %currentDate,"w")
    map_kml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?><kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>Robot Path</name><Style id=\"yellowLineGreenPoly\"><LineStyle><color>7f00ffff</color><width>4</width></LineStyle><PolyStyle><color>7f00ff00</color></PolyStyle></Style><Placemark><name>Robot Path</name><description>The path the robot takes</description><styleUrl>#yellowLineGreenPoly</styleUrl><LineString><extrude>0</extrude><tessellate>1</tessellate><altitudeMode>clampToGround</altitudeMode><coordinates>")



def gps_head_callback(gps_data):
    gps_head = str(gps_data.longitude) + ',' + str(gps_data.latitude) + '\n'
    map_kml.write(gps_head)
    print("Saving...")

'''def gps_tail_callback(gps_data):
    gps_tail = str(gps_data.longitude) + ',' + str(gps_data.latitude) + '\n'
    map_kml.write(gps_tail)
    print("Saving...")'''

if __name__ == "__main__":
    rospy.init_node('overlay', anonymous=True)
    start_mapping()
    rospy.Subscriber('gps_head', NavSatFix, gps_head_callback)
    rospy.spin()
    #rospy.Subscriber('gps_tail', NavSatFix, gps_tail_callback)
    map_kml.write("</coordinates></LineString></Placemark></Document></kml>")
    print("done")
