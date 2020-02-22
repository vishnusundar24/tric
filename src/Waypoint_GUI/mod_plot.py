
from __future__ import print_function
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from matplotlib.text import Text
from matplotlib.image import AxesImage
import numpy as np
from numpy.random import rand
from waypoint_modifier import *


# TODO: handle exception that is raised in waypoint_modifier.find_node() when
#       multiple points are accidently selected

"""
CLASS - WaypointGUI
This class handles the mosue events that come from the user. Coordinate points
can be deleted, moved, or created. There are options to allow panning the screen
by switching modes via a keyboard button press.

    ATTRIBUTES: basis (coordinate), the basis is the coordinate point which is selected
                site,
                state (integer), the state is the current mode the mouse is in, be it
                       deleting, inserting, or panning
                was_right_clicked (boolean), keeps track of whether or not a
                                             right click happened.


    METHODS: __init__: constructor
             pick_point: Moves the selected coordinate point to the chosen location
             delete: Handles removing coordinate points from the screen
             remove: Helper method for handling the on pick events in delete
             press: Pairs keyboard input with an integer to help interpret what
                    action should be taken for what button was pressed.
             onclick: Places a new coordinate point and moves it.
"""




class WaypointGUI():
    fig, ax1 = plt.subplots(1, 1)
    mod = modify_waypoint_path('../waypoint_calibration/waypoint.yml') #".." references parent directory

    """
    Constructor, initializes some attributes, handles mouse event types such as key presses,
                 mouse clicks, and pick events.

    Consumes: Nothing
    Produces: Nothing
    """
    def __init__(self):
        self.basis = None
        self.site = None
        self.state = 0

        # NOTE:  one main callback for each event type
        self.fig.canvas.mpl_connect("pick_event", self.delete)
        self.fig.canvas.mpl_connect("button_press_event", self.alter)
        self.fig.canvas.mpl_connect("button_release_event", self.select_site)
        self.fig.canvas.mpl_connect("key_press_event", self.press)


    """
    press, interprets keys pressed on the keyboard and allows the user to
           switch between modes such as insertion or panning.

    Consumes: An event (keyboard button press)
    Produces: Nothing
    """
    # NOTE:  state change should only effect insertion or moving
    def press(self, event):
        if event.key == 'n':
            #select basis state
            self.state = 1
        elif event.key == 'm':
            #place new point state
            self.state = 2


    """
    pick_point, moves the coordinate point to the desired location.

    Consumes: An event (mouse action)
    Produces: Nothing
    """
    # NOTE:  this pick event should be selecting the relevent point
    def pick_point(self, event):
        if self.was_left_clicked:
            if isinstance(event.artist, Line2D):
                self.was_left_clicked = False
                thisline = event.artist
                xdata = thisline.get_xdata()
                ydata = thisline.get_ydata()
                ind = event.ind
                self.basis = [np.take(xdata, ind), np.take(ydata, ind)]

    """
    delete, deletes the coordinate point clicked on by the user. Must use a
            right click.

    Consumes: An event (mouse action)
    Produces: Nothing
    """
    # NOTE: removes point
    def delete(self, event):
        if event.mouseevent.button == 3:
            if isinstance(event.artist, Line2D):
                thisline = event.artist
                xdata = thisline.get_xdata()
                ydata = thisline.get_ydata()
                ind = event.ind
                self.basis = [np.take(xdata, ind), np.take(ydata, ind)]
                self.mod.remove(self.basis)
                self.mod.save()
                plot_list = self.mod.return_list()
                plt.cla()
                for i in range(len(plot_list[0])-1):
                    dx, dy = plot_list[0][i+1] - plot_list[0][i], plot_list[1][i+1] - plot_list[1][i]
                    self.ax1.arrow(plot_list[0][i], plot_list[1][i], dx, dy, head_width=.5, head_length=1)
                line = self.ax1.plot(plot_list[0], plot_list[1], 'o', picker=5)
                plt.draw()
                self.was_right_clicked = False


    """
    select_site, determines the location for a new or moved point
    Consumes: A mouse event
    Produces: Nothing
    """
    # NOTE: runs on a mouse release event
    def select_site(self, event):
        if event.button == 1:
            x, y = event.xdata, event.ydata
            self.site = [x,y]
            if self.state == 1:
                self.mod.insert(self.basis[0], self.basis[1], self.site[0], self.site[1])
                self.basis = self.site
            elif self.state == 2:
                self.mod.move(self.basis, self.site)

            self.mod.save()
            plot_list = self.mod.return_list()
            plt.cla()
            for i in range(len(plot_list[0])-1):
                dx, dy = plot_list[0][i+1] - plot_list[0][i], plot_list[1][i+1] - plot_list[1][i]
                self.ax1.arrow(plot_list[0][i], plot_list[1][i], dx, dy, head_width=.5, head_length=1)
            line = self.ax1.plot(plot_list[0], plot_list[1], 'o', picker=5)
            plt.draw()


    def alter(self, event):
        if event.button == 1:
            self.was_left_clicked = True
            self.was_right_clicked = False
            self.fig.canvas.mpl_connect("pick_event", self.pick_point)
        elif event.button == 3: # 3 is an enumeration for right click
            self.was_right_clicked = True
            self.was_left_clicked = False
            self.fig.canvas.mpl_connect("pick_event", self.delete)
        return
