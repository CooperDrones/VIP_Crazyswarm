from anytree import Node, RenderTree
from anytree.exporter import DotExporter

launch   = Node("<launch_file>")
cf       = Node("crazyflie<no>", parent=launch)

hover    = Node("hoverStiff", parent=cf)
traj     = Node("trajTracking", parent=cf)
land     = Node("land", parent=cf)

altitude = Node("AltitudeControllerPhys", parent=hover)
xy_hover = Node("XYControllerPhys", parent=hover)
yaw      = Node("YawControllerPhys", parent=hover)

altitude = Node("AltitudeControllerPhys", parent=traj)
xy_traj  = Node("XYControllerTrajPhys", parent=traj)
yaw      = Node("YawControllerPhys", parent=traj)

xy_hover = Node("XYControllerPhys", parent=land)
yaw      = Node("YawControllerPhys", parent=land)

traj_gen = Node("TrajGenerator", parent=cf)
wave     = Node("wave_traj", parent=traj_gen)
circle   = Node("circle_traj", parent=traj_gen)

# graphviz needs to be installed for the next line!
DotExporter(launch).to_picture("plots/tree.png")
