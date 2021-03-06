# #!/usr/bin/env python
# # license removed for brevity
# import matplotlib.pyplot as plt
# import rospy
# from uav_msgs.msg import VehicleState, VehicleStateArray, TargetWaypoints, Chat
# import numpy as np
# from matplotlib.animation import FuncAnimation
# import math

# class Visualiser:
#     def __init__(self):
#         self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3)
        
#         self.ln1, = self.ax1.plot([], [], '.', markersize=1, label='ego trajectory')
#         self.ax1.legend()
#         self.x_data, self.y_data = [], []

#         self.ln2, = self.ax2.plot([], [], '-', markersize=1, label='ego velocity')
#         self.ax2.legend()
#         self.vel_x_data, self.vel_y_data = [], []

#         self.ln3, = self.ax3.plot([], [], '-', markersize=1, label='distance error')
#         self.ax3.legend()
#         self.err_x_data, self.err_y_data = [], []


#         self.waypoint_x = 0.0
#         self.waypoint_y = 0.0

#     def PlotInit(self):
#         self.ax1.set_xlim(-30, 0)
#         self.ax1.set_ylim(0, 50)

#         self.ax2.set_xlim(0, 800)
#         self.ax2.set_ylim(0, 20)

#         self.ax3.set_xlim(0, 800)
#         self.ax3.set_ylim(0, 10)
#         return self.ln1
    
#     def EgoTrajectoryCallback(self, ego_state):
#         point = ego_state.local_posestamped.pose.position
#         self.x_data.append(point.x)
#         self.y_data.append(point.y)

#         vel_ms = math.sqrt(math.pow(ego_state.velocity.linear.x, 2.0) + math.pow(ego_state.velocity.linear.y, 2.0))
#         vel_kmh = vel_ms * 3.6

#         self.vel_y_data.append(vel_kmh)
#         length = len(self.vel_y_data)
#         self.vel_x_data.append(length + 1)

#         diff_x = point.x - self.waypoint_x
#         diff_y = point.y - self.waypoint_y
#         diff_dist = math.sqrt(math.pow(diff_x, 2.0) + math.pow(diff_y, 2.0))

#         self.err_y_data.append(diff_dist)
#         length = len(self.err_y_data)
#         self.err_x_data.append(length + 1)


#     def WaypointsCallback(self, waypoints):
#         poses = waypoints.local.poses
#         last_pose = poses[-1]
#         self.waypoint_x = last_pose.position.x
#         self.waypoint_y = last_pose.position.y

#     def UdatePlot(self, frame):
#         self.ln1.set_data(self.x_data, self.y_data)
#         self.ln1.set_color('blue')

#         self.ln2.set_data(self.vel_x_data, self.vel_y_data)
#         self.ln2.set_color('blue')

#         self.ln3.set_data(self.err_x_data, self.err_y_data)
#         self.ln3.set_color('blue')
#         return self.ln1


# rospy.init_node('eval')
# vis = Visualiser()
# sub = rospy.Subscriber('/control/generate_waypoints_node/ego_state', VehicleState, vis.EgoTrajectoryCallback)
# sub = rospy.Subscriber('/control/generate_waypoints_node/target_waypoints', TargetWaypoints, vis.WaypointsCallback)

# ani = FuncAnimation(vis.fig, vis.UdatePlot, init_func=vis.PlotInit)
# plt.show(block=True) 



# #!/usr/bin/env python
# # license removed for brevity
# import matplotlib.pyplot as plt
# import rospy
# from uav_msgs.msg import VehicleState, VehicleStateArray, TargetWaypoints, Chat
# import numpy as np
# from matplotlib.animation import FuncAnimation
# import math

# class Visualiser:
#     def __init__(self):
#         self.fig, self.ax1 = plt.subplots()
        
#         self.ln1, = self.ax1.plot([], [], '.', markersize=1, label='ego trajectory')
#         self.ax1.legend()
#         self.x_data, self.y_data = [], []

#     def PlotInit(self):
#         self.ax1.set_xlim(-30, 0)
#         self.ax1.set_ylim(0, 50)
#         return self.ln1
    
#     def EgoTrajectoryCallback(self, ego_state):
#         point = ego_state.local_posestamped.pose.position
#         self.x_data.append(point.x)
#         self.y_data.append(point.y)

#     def UdatePlot(self, frame):
#         self.ln1.set_data(self.x_data, self.y_data)
#         self.ln1.set_color('blue')
#         return self.ln1

# rospy.init_node('eval')
# vis = Visualiser()
# sub = rospy.Subscriber('/control/generate_waypoints_node/ego_state', VehicleState, vis.EgoTrajectoryCallback)

# ani = FuncAnimation(vis.fig, vis.UdatePlot, init_func=vis.PlotInit)
# plt.show(block=True) 




###----------------------------------------------------------------------------------------------------



#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from  matplotlib.pyplot import plot
import rospy
from uav_msgs.msg import VehicleState, VehicleStateArray, TargetWaypoints, Chat
import numpy as np
from matplotlib.animation import FuncAnimation
import math

class Visualiser:
    def __init__(self):
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(1, 4)
        
        self.ln1, = self.ax1.plot([], [], '.', markersize=1, label='ego trajectory')
        self.ax1.legend()
        self.x_data, self.y_data = [], []

        self.ln2, = self.ax2.plot([], [], '.', markersize=1, label='waypoints')
        self.ax2.legend()
        self.wp_x_data, self.wp_y_data = [], []

        self.ln3, = self.ax3.plot([], [], '-', markersize=1, label='ego velocity')
        self.ax3.legend()
        self.vel_x_data, self.vel_y_data = [], []

        self.ln4, = self.ax4.plot([], [], '-', markersize=1, label='distance error')
        self.ax4.legend()
        self.err_x_data, self.err_y_data = [], []

        self.waypoint_x = 0.0
        self.waypoint_y = 0.0

    def PlotInit(self):
        self.ax1.set_xlim(-39, 0)
        self.ax1.set_ylim(0, 45)

        self.ax2.set_xlim(-39, 0)
        self.ax2.set_ylim(0, 45)

        self.ax3.set_xlim(0, 1600)
        self.ax3.set_ylim(0, 12)

        self.ax4.set_xlim(0, 1600)
        self.ax4.set_ylim(0, 5)
        return self.ln1
    
    def EgoStatesCallback(self, ego_state):
        point = ego_state.local_posestamped.pose.position
        self.x_data.append(point.x)
        self.y_data.append(point.y)

        vel_ms = math.sqrt(math.pow(ego_state.velocity.linear.x, 2.0) + math.pow(ego_state.velocity.linear.y, 2.0))
        vel_kmh = vel_ms * 3.6

        self.vel_y_data.append(vel_kmh)
        length = len(self.vel_y_data)
        self.vel_x_data.append(length + 1)

        diff_x = point.x - self.waypoint_x
        diff_y = point.y - self.waypoint_y
        diff_dist = math.sqrt(math.pow(diff_x, 2.0) + math.pow(diff_y, 2.0))

        self.err_y_data.append(diff_dist)
        length = len(self.err_y_data)
        self.err_x_data.append(length + 1)

    def WaypointsCallback(self, waypoints):
        poses = waypoints.local.poses
        last_pose = poses[-1]
        self.waypoint_x = last_pose.position.x
        self.waypoint_y = last_pose.position.y

        self.wp_x_data.append(self.waypoint_x)
        self.wp_y_data.append(self.waypoint_y)


    def UdatePlot(self, frame):
        self.ln1.set_data(self.x_data, self.y_data)
        self.ln1.set_color('blue')

        self.ln2.set_data(self.wp_x_data, self.wp_y_data)
        self.ln2.set_color('red')

        self.ln3.set_data(self.vel_x_data, self.vel_y_data)
        self.ln3.set_color('blue')

        self.ln4.set_data(self.err_x_data, self.err_y_data)
        self.ln4.set_color('blue')
        return self.ln1


rospy.init_node('eval')
vis = Visualiser()
sub = rospy.Subscriber('/control/generate_waypoints_node/ego_state', VehicleState, vis.EgoStatesCallback)
sub = rospy.Subscriber('/control/generate_waypoints_node/target_waypoints', TargetWaypoints, vis.WaypointsCallback)

ani = FuncAnimation(vis.fig, vis.UdatePlot, init_func=vis.PlotInit)
plt.show(block=True) 

###----------------------------------------------------------------------------------------------------

# #!/usr/bin/env python
# # license removed for brevity
# import matplotlib.pyplot as plt
# from  matplotlib.pyplot import plot
# import rospy
# from uav_msgs.msg import VehicleState, VehicleStateArray, TargetWaypoints, Chat
# import numpy as np
# from matplotlib.animation import FuncAnimation
# import math

# class Visualiser:
#     def __init__(self):
#         self.fig, self.ax1 = plt.subplots()
        
#         self.ln1, = self.ax1.plot([], [], '.', markersize=1, label='ego trajectory')
#         self.ax1.legend()
#         self.x_data, self.y_data = [], []


#     def PlotInit(self):
#         self.ax1.set_xlim(-39, 0)
#         self.ax1.set_ylim(0, 45)
#         return self.ln1
    
#     def EgoStatesCallback(self, ego_state):
#         point = ego_state.local_posestamped.pose.position
#         self.x_data.append(point.x)
#         self.y_data.append(point.y)

#     def UdatePlot(self, frame):
#         self.ln1.set_data(self.x_data, self.y_data)
#         self.ln1.set_color('blue')
#         return self.ln1

# rospy.init_node('eval')
# vis = Visualiser()
# sub = rospy.Subscriber('/control/generate_waypoints_node/ego_state', VehicleState, vis.EgoStatesCallback)

# ani = FuncAnimation(vis.fig, vis.UdatePlot, init_func=vis.PlotInit)
# plt.show(block=True) 