'''
GUI to graphicaly move waypoints, using modify_waypoint_path class

Author: Ansel Tessier (ansel.tessier@gmail.com)
'''
import Tkinter as tk
from Tkinter import *
import gps_toolbox
root = tk.Tk()


def paint(event):
    python_green = "#476042"
    x1, y1 = (event.x - 1), (event.y - 1)
    x2, y2 = (event.x + 1), (event.y + 1)
    w.create_oval(x1, y1, x2, y2, fill=python_green)

def motion(event):
    x, y = event.x, event.y
    print('{}, {}'.format(x, y))




root.bind('<Motion>', motion)
w = Canvas(root)
w.pack(expand=YES, fill=BOTH)
w.bind("<B1-Motion>", paint)
root.mainloop()
