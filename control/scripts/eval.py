#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
import math

from uav_msgs.msg import VehicleState, VehicleStateArray, TargetWaypoints, Chat

class Visualiser:
    def __init__(self):
        self.x_data, self.y_data = [], []
        self.wp_x_data, self.wp_y_data = [], []
        self.vel_x_data, self.vel_y_data = [], []
        self.err_x_data, self.err_y_data = [], []

        self.waypoint_x = 0.0
        self.waypoint_y = 0.0

        self.fig, self.axs = plt.subplots(1, 3)
        
        self.axs[0].set_xlabel('x [m]')  # Add an x-label to the axes.
        self.axs[0].set_ylabel('y [m]')  # Add a y-label to the axes.
        self.axs[0].set_title("Trajectory")  # Add a title to the axes.

        self.axs[1].set_xlabel('v [km/h]')  # Add an x-label to the axes.
        self.axs[1].set_ylabel('step []')  # Add a y-label to the axes.
        self.axs[1].set_title("Ego Velocity")  # Add a title to the axes.

        self.axs[2].set_xlabel('s [m]')  # Add an x-label to the axes.
        self.axs[2].set_ylabel('step []')  # Add a y-label to the axes.
        self.axs[2].set_title("Distance Error")  # Add a title to the axes.

    def PlotInit(self):
        self.axs[0].set_xlim(-39, 0)
        self.axs[0].set_ylim(0, 45)

        self.axs[1].set_xlim(0, 1600)
        self.axs[1].set_ylim(0, 12)

        self.axs[2].set_xlim(0, 1600)
        self.axs[2].set_ylim(0, 5)
    
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
        for ax in self.axs:
            ax.clear()

        self.axs[0].set_xlabel('x [m]')  # Add an x-label to the axes.
        self.axs[0].set_ylabel('y [m]')  # Add a y-label to the axes.
        self.axs[0].set_title("Trajectory")  # Add a title to the axes.
        self.axs[0].set_xlim(-39, 0)
        self.axs[0].set_ylim(0, 45)
        self.axs[0].plot(self.x_data, self.y_data, label='ego trajectory')  # Plot some data on the axes.
        self.axs[0].plot(self.wp_x_data, self.wp_y_data, label='waypoints')  # Plot more data on the axes...
        self.axs[0].legend()  # Add a legend.
        
        self.axs[1].set_ylabel('v [km/h]')  # Add an x-label to the axes.
        self.axs[1].set_xlabel('step []')  # Add a y-label to the axes.
        self.axs[1].set_title("Ego Velocity")  # Add a title to the axes.
        self.axs[1].set_xlim(0, 1600)
        self.axs[1].set_ylim(0, 12)
        self.axs[1].plot(self.vel_x_data, self.vel_y_data)  # Plot some data on the axes.

        self.axs[2].set_ylabel('s [m]')  # Add an x-label to the axes.
        self.axs[2].set_xlabel('step []')  # Add a y-label to the axes.
        self.axs[2].set_title("Distance Error")  # Add a title to the axes.
        self.axs[2].set_xlim(0, 1600)
        self.axs[2].set_ylim(0, 5)
        self.axs[2].plot(self.err_x_data, self.err_y_data)  # Plot some data on the axes.


rospy.init_node('eval')
vis = Visualiser()
sub = rospy.Subscriber('/control/generate_waypoints_node/ego_state', VehicleState, vis.EgoStatesCallback)
sub = rospy.Subscriber('/control/generate_waypoints_node/target_waypoints', TargetWaypoints, vis.WaypointsCallback)

ani = FuncAnimation(vis.fig, vis.UdatePlot, init_func=vis.PlotInit)
plt.show(block=True) 