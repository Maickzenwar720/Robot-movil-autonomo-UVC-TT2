#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from copy import deepcopy
from signal import signal, SIGINT

def main():

    def signal_handler(signal, frame):
        print("Programa detenido por Ctrl + C")
        navigator.cancelTask() # Detener la navegación si está en curso
        rclpy.shutdown()

    signal(SIGINT, signal_handler)

    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    #SALON122
    # inspection_route = [
    # [0.5239333919684425, -0.04016919344452539, -0.04946361476893275, 0.9987759262286965],
    # [2.288869743421497, -0.10662003806034111, 0.01958094411461696, 0.9998082749345397],
    # [5.746625053899792, 0.0445940377263037, 0.7173399373712738, 0.6967233412568987],
    # [5.690542982085627, 1.9597827090390145, 0.703097149528821, 0.7110938041668249],
    # [5.499816561257998, 3.697094413212218, -0.9991539390062488, 0.041126708697599344],
    # [2.8707860080941474, 3.630116089876883, -0.9978345149083222, 0.06577446964950291],
    # [-0.35049240141486177, 3.4652461719161334, -0.7204153139653405, 0.6935429153298445],
    # [-0.2822994311672727, 1.706913373330805, -0.6838402615178099, 0.729631754193342],
    # [-0.1872480420423569, -0.13417314364995367, 0.04142593557350351, 0.9991415774863239]]
    # PESADOS
    inspection_route = [ # simulation points
        #[-1.5695, -13.9715, -0.0135, 0.9999],
        [-0.8294, -13.7828, 0.0435, 0.9999],
        [0.389, -13.7717, 0.0124, 0.9999],
        [1.955, -13.7338, -0.0023, 0.9999],
        [2.6956, -12.8043, 0.7113, 0.7028],
        [1.7358, -11.5705, -0.9996, 0.0266],
        [0.7148, -11.5573, -0.9979, 0.0094],
        [-0.6557, -11.5514, -0.9999, 0.0112],
        [-1.4687, -10.4233, 0.7075, 0.7066],
        [-0.6905, -9.3424, -0.0166, 0.9999],
        [0.6139, -9.3763, -0.0105, 0.9999],
        [1.5947, -9.4099, 0.0173, 0.9999],
        [2.7162, -8.6583, 0.7223, 0.6915],
        [1.8404, -6.9831, 0.9999, 0.0091],
        [0.6634, -6.9536, 0.9989, 0.0464],
        [-0.4711, -7.0192, -0.9999, 0.026],
        [-1.511, -8.126, -0.6704, 0.7419],
        [-1.5695, -13.9715, -0.0135, 0.9999]]
    #AUDITORIO
    # inspection_route = [
    # [4.145319812707247, 5.448404642135191, -0.776729189155633, 0.6298347138048465],
    # [2.0141060852986414, 5.386583269698065, -0.8319755045349897, 0.5548123645465638],
    # [0.4634577612906601, -1.3665042995817793, -0.24310170399862693, 0.9700008049032557],
    # [2.5878812144460364, -1.9117983638498257, 0.5096083558213717, 0.8604064874680097],
    # [4.145319812707247, 5.448404642135191, -0.776729189155633, 0.6298347138048465]]

    #SIMULACION
    # inspection_route = [ # simulation points
    #     #[-1.5695, -13.9715, -0.0135, 0.9999],
    #     [0.0294, -3.6004, -0.0216, 0.9997],
    #     [2.6826, -3.6004, -0.6862, 0.7273],
    #     [2.6826, -7.3047, -0.9997, 0.0327],
    #     [-0.4078, -7.3047, 0.7174, 0.6966],
    #     [-0.4078, -3.6004, 0.9997, 0.0231],
    #     [-3.4027, -3.6004, -0.7226, 0.6912],
    #     [-3.4027, -7.3047, 0.9997, 0.0851],
    #     [-6.3159, -7.3047, 0.7265, 0.6871],
    #     [-6.3159, -3.6004, 0.0609, 0.9991],
    #     [0.01, -3.6004, 0.7162, 0.6978],
    #     [0.001, 0.001, 0.0314, 0.9995]]


    # # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = -1.5695
    # initial_pose.pose.position.y = -13.9715
    # initial_pose.pose.orientation.z = -0.0135
    # initial_pose.pose.orientation.w = 0.9999
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    while rclpy.ok():

        # Send our route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_pose.pose.orientation.z = pt[2]
            inspection_pose.pose.orientation.w = pt[3]
            inspection_points.append(deepcopy(inspection_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)

        # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
        # Simply print the current waypoint ID for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Inspection of shelves complete! Returning to start...')
        elif result == TaskResult.CANCELED:
            print('Inspection of shelving was canceled. Returning to start...')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Inspection of shelving failed! Returning to start...')

        
        exit(0)




if __name__ == '__main__':
    main()

