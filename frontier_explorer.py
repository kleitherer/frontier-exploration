#!/usr/bin/env python3


import rclpy                    
from rclpy.node import Node     # ROS2 node baseclass

import numpy as np
from scipy.signal import convolve2d

from nav_msgs.msg import OccupancyGrid
from asl_tb3_msgs.msg import TurtleBotState
from std_msgs.msg import Bool

from asl_tb3_lib.occupancy import StochOccupancyGrid2D

def explore(occupancy):
    """Return frontier states in (x, y) coordinates. Copy and paste from part 2"""

    window_size = 13

    # OCCUPIED if probability > threshold
    occupied_mask = (occupancy.probs > occupancy.thresh)

    # UNKNOWN if explicitly -1 OR below threshold
    if np.any(occupancy.probs < 0):
        unknown_mask = (occupancy.probs < 0)
    else:
        unknown_mask = (occupancy.probs <= occupancy.thresh)

    free_mask = ~(occupied_mask | unknown_mask)

    kernel = np.ones((window_size, window_size), float)
    total_neighbors = kernel.size

    unknown_neighbors  = convolve2d(unknown_mask.astype(float),  kernel, mode='same')
    occupied_neighbors = convolve2d(occupied_mask.astype(float), kernel, mode='same')
    free_neighbors     = convolve2d(free_mask.astype(float),     kernel, mode='same')

    h1 = (unknown_neighbors / total_neighbors) >= 0.20
    h2 = (occupied_neighbors == 0)
    h3 = (free_neighbors / total_neighbors) >= 0.30

    frontier_mask = h1 & h2 & h3 & free_mask

    ys, xs = np.where(frontier_mask)

    frontier_states = np.array([
        occupancy.grid2state(np.array([x, y]))
        for x, y in zip(xs, ys)
    ])

    return frontier_states

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__("frontier_explorer")

        self.map_msg = None
        self.state_msg = None
        self.busy = False  

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        """
        Adding to the previous point, the navigator publishes a boolean message to the /nav_success topic. A
        true message is published when the robot reaches the commanded navigation pose, and a false message
        is published when planning (or re-planning) fails. Your exploration node needs to listen to this topic
        to figure out when to send out the next navigation command.
        """
        self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)

        self.nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)



    def map_callback(self, msg):
        self.map_msg = msg

    def state_callback(self, msg):
        self.state_msg = msg

    def nav_success_callback(self, msg: Bool):
        self.busy = False
        self.get_logger().info("Navigator finished. Choosing next frontier...")
        self.send_next_goal()
    
    def send_next_goal(self):

        if self.busy:
            return
        if self.map_msg is None or self.state_msg is None:
            return

        msg = self.map_msg
        probs = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([
                msg.info.origin.position.x,
                msg.info.origin.position.y
            ]),
            window_size=13,
            probs=probs
        )

        frontiers = explore(occupancy)

        if len(frontiers) == 0:
            self.get_logger().info("No frontiers found. Exploration complete?")
            return

        # pick the closest state to the robot
        cur = np.array([self.state_msg.x, self.state_msg.y])
        dists = np.linalg.norm(frontiers - cur, axis=1)
        goal_xy = frontiers[np.argmin(dists)]

        goal_msg = TurtleBotState()
        goal_msg.x = float(goal_xy[0])
        goal_msg.y = float(goal_xy[1])
        goal_msg.theta = 0.0  

        self.nav_pub.publish(goal_msg)
        self.busy = True

        self.get_logger().info(
            f"Sent new frontier goal: ({goal_msg.x:.2f}, {goal_msg.y:.2f})"
        )