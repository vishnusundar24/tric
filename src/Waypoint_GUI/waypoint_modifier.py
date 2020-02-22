#!/usr/bin/env python
'''
Class to modify an existing waypoint path, both by updating the position of
established points inserting new points, and deleting old points

**Please specify path to waypoint yml file with command line arg 1**

Author: Ansel Tessier (ansel.tessier@gmail.com)

    Supporting Class node:
        Attributes:
            data: the value a particular node is responcible for
            next: the node that this node refrences

        Methods:
            NA

Attributes:
    head: head node of linked list
    tail: tail node of linked list
    dautm: gps point that is the origin of the workspace, cartesian points are
    relatave to it


Methods:
    read_data: takes in the existing waypoints form yml file, and inserts them
    into it

    cartesian_from_gps (Author, Adam Stager): takes a gps lat/lon coordinate,
    and converts it into a cartesian coordinate relative to a datum (a gps
    point located at the origin of the workspace)


    node: supporting class for linked list data structure, may not be needed, will
    be addressed soon after 1/3/20


    gps_from_cartesian: converts a cartesian point relative to the datum to a
    tuple (lat, lon)


    save: saves waypoints in a new dictionary that then oveewrites the specified waypoint file
'''
# TODO: check to see if datum nessicarily has to be the min lat/lon,
#       otherwise define it as the first waypoint
import sys
import yaml
from gps_tools.gps_toolbox import *
import math
import Tkinter



class modify_waypoint_path:

    #constructor
    def __init__(self, file):
        self.has_data = False
        self.head = None
        self.tail = None
        self.path_file = file
        self.datum = None

    #supporting class for linked list, this approach is being used to make it easier to insert new waypoints
    class node():
        #constructor
        def __init__(self, data=None):
            self.data = data
            self.next = None


    #convert to cartesian from gps
    # TODO: add this as a method in GpsToolbox (re-install package)
    #arguments: wayopint: GpsToolbox object, datum: GpsToolbox object
    #returns: list: [x_postion, y_position]
    def cartesian_from_gps(self, waypoint, datum):
        '''Return x,y position (in meters) relative to a datum (local origin)'''
        min_lat = datum.lat
        min_lon = datum.lon
        obs_lat = waypoint.lat
        obs_lon = waypoint.lon

        R = 6371  # radius of earth at equator (km)
        alpha = obs_lat  # alpha in degrees
        r = R*math.cos(alpha*math.pi/180)
        l = r*math.pi/180
        L = R*math.pi/180

        x_km = L*(obs_lat - min_lat)
        y_km = l*(obs_lon - min_lon)
        x_m = x_km*1000 # convert km to meters
        y_m = y_km*1000
        position = [x_m, y_m]
        return position


    #arguments: NA
    #returns: NA
    #Prints out linked list
    def print_list(self):
        printval = self.head
        while printval is not None:
            print (printval.data)
            printval = printval.next

    def return_list(self):
        format = []
        format.append([])
        format.append([])
        if self.has_data:
            curr = self.head
            while curr is not None:
                format[0].append(curr.data[0])
                format[1].append(curr.data[1])
                curr = curr.next
            return format
        else:
            print("please call read_data first")
            return None



    #read in wayopint dictionary
    #arguments: NA
    #returns: NA
    #convert dict to linked list
    def read_data(self):
        self.has_data = True
        with open(self.path_file, 'r') as stream:
            waypoint_dict = yaml.load(stream)
        self.datum = waypoint_dict.get(0)
        for i in range(len(waypoint_dict)):
            new_node = self.node(waypoint_dict[i])
            if i == 0:
                self.head = new_node
                self.tail = new_node
            else:
                self.tail.next = new_node
                self.tail = new_node

    #arguments: NA
    #returns: NA
    #Method that callc cartesian_from_gps on every elemnet in tht linked list
    def list_to_cartesian(self):
        curr = self.head
        while curr is not None:
            curr.data = self.cartesian_from_gps(curr.data, self.datum)
            curr = curr.next

    #arguments: tuple (x,y)
    #linear search of linked list
    def find_node(self, x_y, get_prev = False):
        thd = .001
        curr = self.head
        x = x_y[0]
        y = x_y[1]
        try:
            if get_prev:
                while curr is not None:
                    deltax = x - curr.next.data[0]
                    deltay = y - curr.next.data[1]

                    if (abs(deltax) < thd) and (abs(deltay) < thd):
                        return curr
                    curr = curr.next
            else:
                while curr is not None:
                    deltax = x - curr.data[0]
                    deltay = y - curr.data[1]

                    if (abs(deltax) < thd) and (abs(deltay) < thd):
                        return curr
                    curr = curr.next
            print("No wayopint at this position")
            print(x_y)
        except:
            print("Multiple points selected")

    #inserts element new into linked list after element previous
    def insert_waypoint(self, new_data, before):
        new_node = self.node(new_data)
        new_node.next = before.next
        before.next = new_node

    #arguments tuple(x,y), tuple(x,y)
    def insert(self, lastx, lasty, newx, newy):
        new = [newx, newy]
        last = [lastx, lasty]
        self.insert_waypoint(new, self.find_node(last))

    def remove(self, x_y):
        before_remove = self.find_node(x_y, get_prev=True)
        before_remove.next = before_remove.next.next
        #before_remove.next.next = None

    def move(self, old, new):
        to_change = self.find_node(old)
        to_change.data = new


    #def update(self, )


    #arguments: tuple (x,y), GpsToolbox object
    #returns: tuple (lat, lon)
    #converts an x,y cartesian point to a lat, lon gps point
    def gps_from_cartesian(self, x_y, datum):
        min_lat = datum.lat
        min_lon = datum.lon
        R = 6371 #radius of earth
        x_km = x_y[0] / 1000
        y_km = x_y[1] / 1000
        lat = ((180*x_km)/(R*math.pi)) + min_lat
        lon = ((y_km*180)/(math.pi*R*math.cos(lat*(math.pi/180)))) + min_lon
        return (lat, lon)

    #arguments: NA
    #returns: NA
    #converts linked list back to dictionary and saves to file
    def save(self):
        i = 0
        new_waypoints = {}
        curr = self.head
        while curr is not None:
            s = self.gps_from_cartesian(curr.data, self.datum)
            new_waypoints[i] = GpsToolbox(float(s[0]),float(s[1]))
            i += 1
            curr = curr.next
        with open(self.path_file, 'w') as outfile:
            yaml.dump(new_waypoints, outfile)
'''mod = modify_waypoint_path('./waypoint_calibration/georgetown_small_loop.yml')
mod.read_data()
mod.list_to_cartesian()
mod.save()'''
