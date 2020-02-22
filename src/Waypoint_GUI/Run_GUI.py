from mod_plot import *


"""
launch, a bunch of code used to setup the window for the application. This makes
        the text seen onscreen and read in the data

Consumes: Nothing
Produces: Nothing
"""


def launch():
    waypoint = WaypointGUI() # Instance of the class handling mouse events
    waypoint.mod.read_data()
    waypoint.mod.list_to_cartesian()

    plot_list = waypoint.mod.return_list()
    txt="Right click to delete | Press 'M', then left click & drag to move | Press 'N', then left click & drag for new segment, then Left click for new points "
    txt1="All changes saved automatically"
    txt2 = "Look at WAYPOINT_GUI_DOCS.txt for more information"
    waypoint.fig.text(.5, .04, txt, ha="center")
    waypoint.fig.text(.5, .95, txt1, ha="center")
    waypoint.fig.text(.5, .01, txt2, ha="center")
    waypoint.ax1.plot(plot_list[0], plot_list[1], "o", picker=5)
    for i in range(len(plot_list[0])-1):
        dx, dy = plot_list[0][i+1] - plot_list[0][i], plot_list[1][i+1] - plot_list[1][i]
        waypoint.ax1.arrow(plot_list[0][i], plot_list[1][i], dx, dy, head_width=.5, head_length=1)

    # Display the main window
    plt.show()


if __name__ == "__main__":
    action = str(raw_input("Run: Graphic(g)"))
    if action == "g":
        launch()
