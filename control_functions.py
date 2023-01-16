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


def main():
    
    # Get CV --> ID

    # Initializing
    satisfied = False
    counter = 0

    while not satisfied:
        # Get params from curiculum
    
        go_to(name='OBSERVE')
        
        # CV --> repeat until True 
        add_delay(2.0)

        go_to(name=)
    
        # add_delay
    
        # grasp(close)
    
        # go_to(home)

        # After certain trials as if user's satisfied

        # add_delay()

        # CV --> person's location
    
        # calculateHOLoc(delx, dely, delz) 
    
        # add_delay()
    
        # go_to(HOLocation)
    
        # interaction_model(on)
    
        # if forces > threshold:
            # add_delay()
            # grasp(open)
            # add_delay

        # interaction_model(off)






def go_to(position, orientation, max_linear_speed = 0.5): 
    try:
        rospy.init_node('go_to_cartesian_pose_py')
        limb = Limb()
        tip_name = 'right_hand'
        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=max_linear_speed,
                                     max_linear_accel=6.0,
                                     max_rotational_speed=6.28,
                                     max_rotational_accel=6.28,
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
        #print(joint_angles)
        #rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=None)

        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return False

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
            return True
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                     result.errorId)
            return False
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
        return False

def grasp():
    # if gripper is opened, close
    # if gripper is closed, open
    # if successful,open
    pass

def handover_CV():
    # Function to detect handover location
    pass

def object_CV():
    # Function to detect if the object is kept in the pickup area
    return True

def add_delay(time):

    rospy.sleep(time)

def set_interaction_mode():
    pass


def set_positions(name):
    try:
        pos = [0, 0, 0, 0]
        orientation = [0, 0, 0, 0]

        if name == 'observe':
            pos = [0.6, 0.2, 0.4]
            orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to OBSERVE')

        elif name == 'home':
            pos = [0.4, 0.0, 0.2]
            orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to HOME')
        el

        elif name = 'cv' :
            # Here, get pos from Computer vision
            pos = [1.1, 0.0, 0.3]
            orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to HANDOVER')
        else:
            rospy.logerr('Wrong Position Name, setting next pose to HOME')
            pos = [0.4, 0.0, 0.2]
            orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to HOME')
            return False

        return pos, orientation
    
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
        return False

def is_satisfied():
    # Function to ask the user if they are satisfied with the training
    pass


def get_params():

if __name__ == '__main__':
    main()
