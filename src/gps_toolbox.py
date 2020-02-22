"""
TRIC-Robotics
Author: Ansel Tessier (ansel.tessier@gmail.com)

Class that allows for the holding and processing
of gps coordinates for means of waypoint navigation

    Attributes:
        double lat: the latitude value
        double lon: the longitude value

    Methods:
        __init__
            constructor

        update
            redefines instance variables, to be used when a
            gps unit changes position
            - Arguments: new latitude, new longitude
            - Returns: NA

        getPos
            returns a tuple of latitude and longitude

        dist_to
            caculates the euclidean distance between two
            GopToolbox objects
            - Arguments: gps point
            - Returns: Distance between self and argument points

        get_theta
            caculates angle to a waypoint marker, from
            self and current heading
            - Arguments: gps point, current heading
            - Returns: angle in radians

        cartesian_from_gps
            returns the euclidean coordinates of this gps point relative to some
            origin or datum
            - Arguments: datum
            - Returns: list of x and y coordinates

"""

import math


class GpsToolbox:

    #constructor
    def __init__(self, latPos, lonPos):
        #constructor
        self.lat = latPos
        self.lon = lonPos

    #argumnets: double; latitude, double; longitude
	#returns: NA
    def update(self, latPos, lonPos):
        #updates position of robot
        self.lat = latPos
        self.lon = lonPos

    #argumnets: NA
    #returns: tueple; (lat, long)
    def getPos(self):
        return (self.lat, self.lon)

    #arguments: GpsToolbox object; waypoint
	#returns: double; distance
    def dist_to(self, waypoint):
        #method to caculate distance in meters between two GPS positions
        """caculates distance between two pos objects"""
        delta_lat = (waypoint.lat - self.lat)*111111 #Converts latitude to meters
        delta_lon = (waypoint.lon - self.lon)*111111 #Converts longitude to meters
        distance = math.sqrt(delta_lat**2 + delta_lon**2) #Magnitude of distance to waypoint

        #print('Distance from waypoint:', distance ,' Meters')
        return distance

    #convert to cartesian from gps
    # TODO: add this as a method in GpsToolbox (re-install package)
    #arguments: wayopint: GpsToolbox object, datum: GpsToolbox object
    #returns: list: [x_postion, y_position]
    def cartesian_from_gps(self, datum):
        '''Return x,y position (in meters) relative to a datum (local origin)'''
        min_lat = datum.lat
        min_lon = datum.lon
        self.lat
        self.lon

        R = 6371  # radius of earth at equator (km)
        alpha = self.lat  # alpha in degrees
        r = R*math.cos(alpha*math.pi/180)
        l = r*math.pi/180
        L = R*math.pi/180

        x_km = L*(self.lat - min_lat)
        y_km = l*(self.lon - min_lon)
        x_m = x_km*1000 # convert km to meters
        y_m = y_km*1000
        position = [x_m, y_m]
        return position

    #arguments: GpsToolbox object; waypoint, double; angle(rad)
    #returns: double; angle(rad)
    def get_theta(self, waypoint, heading):
        #method to determine angle between current heading and disired waypoint
        desired_lat = math.radians(waypoint.lat)
        desired_lon = math.radians(waypoint.lon)


        latitude = math.radians(self.lat)
        longitude = math.radians(self.lon)

        delta_lon = desired_lon - longitude

        x = math.cos(desired_lat) * math.sin(delta_lon)
        y = ((math.cos(latitude) * math.sin(desired_lat))
        - (math.sin(latitude)
        * math.cos(desired_lat)
        * math.cos(delta_lon)))

        desired_heading = -math.degrees(math.atan2(y,x))

        #print("X-Value: " + str(x) + " meters")
        #print("Y_Value: " + str(y) + " meters")
        #print("Original Desired Heading" + str(desired_heading))

        if desired_heading < 0:
            desired_heading = desired_heading + 360

        #print("actual heading: " + str(heading))
        #print("desired heading: " + str(desired_heading))
        delta_theta = float(desired_heading) - heading


        if delta_theta > 180:
            delta_theta = delta_theta - 360
        if delta_theta < -180:
            delta_theta = delta_theta + 360
        delta_theta_rad = math.radians(delta_theta)
        return delta_theta_rad
