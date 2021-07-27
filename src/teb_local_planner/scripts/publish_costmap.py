#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de

import rospy
import math
import tf
import pickle
import os
import numpy as np
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from teb_local_planner.msg import VoxGrid
from nav_msgs.msg import OccupancyGrid, MapMetaData
from ros_numpy import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg

import matplotlib.pyplot as plt


def feedback_callback(data):
    global trajectory

    if not data.trajectories:
        trajectory = []
        return
    trajectory = data.trajectories[data.selected_trajectory_idx].trajectory



def load_saved_costmaps(file_path):

    with open(file_path, 'rb') as f:
        collider_data = pickle.load(f)

    return collider_data


def get_collisions_msg(collision_preds, t0, origin0, dl_2D, time_resolution):

    # Define header
    msg = VoxGrid()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = 'odom'

    # Define message
    msg.depth = collision_preds.shape[0]
    msg.width = collision_preds.shape[1]
    msg.height = collision_preds.shape[2]
    msg.dl = dl_2D
    msg.dt = time_resolution
    msg.origin.x = origin0[0]
    msg.origin.y = origin0[1]
    msg.origin.z = -1.0

    #msg.theta = q0[0]
    msg.theta = 0
    msg.data = collision_preds.ravel().tolist()

    return msg


def get_collisions_visu_msg(collision_preds, t0, origin0, dl_2D, visu_T=15):
    '''
    0 = invisible
    1 -> 98 = blue to red
    99 = cyan
    100 = yellow
    101 -> 127 = green
    128 -> 254 = red to yellow
    255 = vert/gris
    '''

    # Define header
    msg = OccupancyGrid()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = 'odom'

    # Define message meta data
    msg.info.map_load_time = rospy.Time.from_sec(t0)
    msg.info.resolution = dl_2D
    msg.info.width = collision_preds.shape[1]
    msg.info.height = collision_preds.shape[2]
    msg.info.origin.position.x = origin0[0]
    msg.info.origin.position.y = origin0[1]
    msg.info.origin.position.z = -0.01
    #msg.info.origin.orientation.x = q0[0]
    #msg.info.origin.orientation.y = q0[1]
    #msg.info.origin.orientation.z = q0[2]
    #msg.info.origin.orientation.w = q0[3]

    # Define message data
    data_array = collision_preds[visu_T, :, :].astype(np.float32)
    mask = collision_preds[visu_T, :, :] > 253
    mask2 = np.logical_not(mask)
    data_array[mask2] = data_array[mask2] * 98 / 253
    data_array[mask2] = np.maximum(1, np.minimum(98, data_array[mask2] * 1.0))
    data_array[mask] = 98  # 101
    data_array = data_array.astype(np.int8)
    msg.data = data_array.ravel()

    return msg


def get_pred_points(collision_preds, t0, origin0, dl_2D, dt):

    # Get mask of the points we want to show

    mask = collision_preds > 0.8 * 255

    nt, nx, ny = collision_preds.shape
    

    t = np.arange(0, nt, 1)
    x = np.arange(0, nx, 1)
    y = np.arange(0, ny, 1)

    tv, yv, xv = np.meshgrid(t, x, y, indexing='ij')
    

    tv = tv[mask].astype(np.float32)
    xv = xv[mask].astype(np.float32)
    yv = yv[mask].astype(np.float32)


    xv = origin0[0] + (xv + 0.5) * dl_2D
    yv = origin0[1] + (yv + 0.5) * dl_2D
    tv *= dt * 0.5

    labels = collision_preds[mask].astype(np.float32) / 255

    return np.stack((xv, yv, tv), 1), labels



def get_pointcloud_msg(new_points, labels):

    # data structure of binary blob output for PointCloud2 data type
    output_dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring'],
                             'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
                             'offsets': [0, 4, 8, 16, 20],
                             'itemsize': 32})

    # fill structured numpy array with points and classes (in the intensity field). Fill ring with zeros to maintain Pointcloud2 structure
    c_points = np.c_[new_points, labels, np.zeros(len(labels))]
    c_points = np.core.records.fromarrays(c_points.transpose(), output_dtype)

    # convert to Pointcloud2 message and publish
    msg = pc2.array_to_pointcloud2(c_points, rospy.get_rostime(), 'odom')

    return msg


def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
    ax_v.cla()
    ax_v.grid()
    ax_v.set_ylabel('Trans. velocity [m/s]')
    ax_v.plot(t, v, '-bx')
    ax_omega.cla()
    ax_omega.grid()
    ax_omega.set_ylabel('Rot. velocity [rad/s]')
    ax_omega.set_xlabel('Time [s]')
    ax_omega.plot(t, omega, '-bx')
    fig.canvas.draw()


def publish_costmap_msg(traj_debug=False):
    global trajectory
    
    if traj_debug:
        topic_name = "/test_optim_node/teb_feedback"
        topic_name = rospy.get_param('~feedback_topic', topic_name)
        rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size=1)

        rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
        rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")
            
        fig, (ax_v, ax_omega) = plt.subplots(2, sharex=True)
        plt.ion()
        plt.show()


    # Load costmaps to publish
    simu_path = '/home/hth/Myhal_Simulation/simulated_runs'
    folder = '2021-06-07-21-44-58'
    pred_file = os.path.join(simu_path, folder, 'logs-' + folder, 'collider_data.pickle')
    collider_data = load_saved_costmaps(pred_file)

    dl = collider_data['dl'][0]
    dt0 = collider_data['dt'][0]
    dt = collider_data['dt'][0]
    pred_times = collider_data['header_stamp']

    # Init
    collision_pub = rospy.Publisher('/test_optim_node/plan_costmap_3D', VoxGrid, queue_size=1)
    visu_pub = rospy.Publisher('/collision_visu', OccupancyGrid, queue_size=1)
    pointcloud_pub = rospy.Publisher('/colli_points', PointCloud2, queue_size=10)
    #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    rospy.init_node("test_obstacle_msg")

    preds_i = 50

    dims = collider_data['dims'][preds_i]
    preds = np.reshape(collider_data['preds'][preds_i], dims)
    
    origin0 = np.copy(collider_data['origin'][0])
    t0 = origin0[2]

    y_0 = -3.0
    vel_y = 0.3
    range_y = 6.0

    r = rospy.Rate(2)  # 10hz
    t = 0.0
    visu_T = 0
    while not rospy.is_shutdown():

        ####################
        # Publishing pred 3D
        ####################

        # Vary The costmap layer to publish
        #visu_T = (visu_T + 1) % preds.shape[0]

        new_origin = np.copy(origin0)

        # new_origin[0] = origin0[0] + 2.0 * np.sin(5.31 * t)
        # new_origin[1] = origin0[1] + 3.0 * np.sin(t)

        new_origin[0] = origin0[0] + 3.5
        new_origin[1] = origin0[1] + 0
        
        # Get messages
        collision_msg = get_collisions_msg(preds, t0, new_origin, dl, dt)
        #visu_msg = get_collisions_visu_msg(preds, t0, new_origin, dl, visu_T)
        points, labels = get_pred_points(preds, t0, new_origin, dl, dt)
        pt_msg = get_pointcloud_msg(points, labels)

        # Publish
        collision_pub.publish(collision_msg)
        #visu_pub.publish(visu_msg)
        pointcloud_pub.publish(pt_msg)

        ###################
        # Plotting feedback
        ###################

        if traj_debug:
            ts = []
            vs = []
            omegas = []
            
            for point in trajectory:
                ts.append(point.time_from_start.to_sec())
                vs.append(point.velocity.linear.x)
                omegas.append(point.velocity.angular.z)
                
            plot_velocity_profile(fig, ax_v, ax_omega, np.asarray(ts), np.asarray(vs), np.asarray(omegas))




        t = t + 0.05
        r.sleep()


if __name__ == '__main__':
    global trajectory
    try:
        trajectory = []
        publish_costmap_msg(traj_debug=False)
    except rospy.ROSInterruptException:
        pass
