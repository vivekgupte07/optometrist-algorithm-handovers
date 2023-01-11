#! /usr/bin/env python


import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb

def get_params():
# [lin_speed, lin_accel, rot_speed, rot_accel, ratio]
    params = [0.3, 0.3, 1.57, 1.57, 1.0]
    return params


def set_params():
    # [lin_speed, lin_accel, rot_speed, rot_accel, ratio]
    params = get_params()
    print(params)
    max_linear_speed = params[0] 
    max_linear_accel = params[1] 
    max_rotational_speed = params[2]
    max_rotational_accel = params[3]
    max_joint_speed_ratio = params[4]

    return max_linear_speed, max_linear_accel, max_rotational_speed, max_rotational_accel, max_joint_speed_ratio


def main():

    retraction = True    
    try:
        while True:
            
            if retraction:
                position = [0.5, -0.0, 0.2]
                orientation = [0.0, 1.0, 0.0, 0.0]
                print("Retracting......")
                retraction = False

            else:
                position = [0.8, -0.3, 0.2]
                orientation = [-0.538174621635, 0.674156024416, -0.3945294507, 0.316588445622]
                print("Reaching....")
                retraction = True
             
            rospy.init_node('go_to_cartesian_pose_py')
            limb = Limb()
            tip_name = 'right_hand'
            traj_options = TrajectoryOptions()
            traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
            traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
            
            max_linear_speed, max_linear_accel, max_rotational_speed, max_rotational_accel, max_joint_speed_ratio = set_params()

            wpt_opts = MotionWaypointOptions(max_linear_speed=0.4,
                                             max_linear_accel=0.7,
                                             max_rotational_speed=1.57,
                                             max_rotational_accel=1.57,
                                             max_joint_speed_ratio=1.0)
           
            waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

            joint_names = limb.joint_names()

            endpoint_state = limb.tip_state(tip_name)

            pose = endpoint_state.pose

            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]

            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
            
            poseStamped = PoseStamped()
            poseStamped.pose = pose

            joint_angles = limb.joint_ordered_angles()
            waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)
            print(joint_angles)
            #rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=None)
            if result is None:
                rospy.logerr('Trajectory FAILED to send')
                return

            if result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                             result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
