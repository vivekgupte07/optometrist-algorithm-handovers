#! /usr/bin/env python

# Class of functions that generate motion to perform Handovers

import rospy
import gripper
from intera_core_msgs.msg import EndpointState
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from intera_interface import Limb
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)


class Handover(object):

    def __init__(self):

        self.pos = [0, 0, 0]
        self.orientation = [0, 0, 0, 0]
        self.success = bool

        self.tip_name = 'right_hand'
        self.limb = Limb()
        self.traj_options = TrajectoryOptions()
        self.poseStamped = PoseStamped()

        self.max_lin_accl = 6.0 
        self.max_rot_speed = 6.28
        self.max_rot_accl = 6.28
        self.speed_ratio = 1.0

        self.set_params()


    def get_params(self):
        # Learning curriculum
        pass

    def set_params(self):
        # get_params()
        # Get new values for the params
        self.max_lin_speed = 0.2
        self.delay = 2.0

        self.del_x = -0.3
        self.del_y = -0.1
        self.del_z = 0.2
 

        self.limb = Limb()
        self.traj_options = TrajectoryOptions()
        self.poseStamped = PoseStamped()


    def go_to(self): 
        
        try:
            self.set_params()
                        
            self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
            
            traj = MotionTrajectory(trajectory_options = self.traj_options, limb = self.limb)

            wpt_opts = MotionWaypointOptions(max_linear_speed=self.max_lin_speed,
                                         max_linear_accel=self.max_lin_accl,
                                         max_rotational_speed=self.max_rot_speed,
                                         max_rotational_accel=self.max_rot_accl,
                                         max_joint_speed_ratio=self.speed_ratio)

            waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self.limb)

            joint_names = self.limb.joint_names()

            endpoint_state = self.limb.tip_state(self.tip_name)

            pose = endpoint_state.pose

            pose.position.x = self.pos[0]
            pose.position.y = self.pos[1]
            pose.position.z = self.pos[2]

            pose.orientation.x = self.orientation[0]
            pose.orientation.y = self.orientation[1]
            pose.orientation.z = self.orientation[2]
            pose.orientation.w = self.orientation[3]

            self.poseStamped = PoseStamped()
            self.poseStamped.pose = pose

            joint_angles = self.limb.joint_ordered_angles()
            waypoint.set_cartesian_pose(self.poseStamped, self.tip_name, joint_angles)
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


    def set_positions(self, name):
        if name == 'OBSERVE':
            self.pos = [0.6, 0.1, 0.4]
            self.orienc = [0, 1, 0, 0]
            self.max_speed = 0.2
            self.success = True
            rospy.loginfo('Set next pose to %s', name)

        elif name == 'HOME':
            self.pos = [0.4, 0.0, 0.2]
            self.orientation = [0, 1, 0, 0]
            self.max_speed = 0.1
            self.success = True
            rospy.loginfo('Set next pose to %s', name)
        
        elif name == 'PICKUP':
            self.pos = [0.6, 0.1, 0.1]
            self.orientation = [0, 1, 0, 0]
            self.max_speed = 0.2
            self.success = True
            rospy.loginfo('Set next pose to %s', name)

        elif name == 'HANDOVER' :
            # Here, get pos from Computer vision
            # is_person, pos = handover_CV()
            # if pos is not None:.... continue else go back to HOME
            # self.calc_HO_loc()
            self.pos = [0.8, 0.0, 0.2]
            self.orientation = [0, 1, 0, 0]
            self.max_speed = 0.2
            self.success = True
            rospy.loginfo('Set next pose to %s', name)

        else:
            rospy.logerr('Wrong Position Name, setting next pose to HOME')
            self.pos = [0.4, 0.0, 0.2]
            self.orientation = [0, 0, 1, 0]
            self.max_speed = 0.1
            rospy.loginfo('Set next pose to HOME')

        self.go_to()


    def grasp(self, act):
        self.set_params()
        gripper()

    def grasp_detection(self):
        #check_grasp_detection()
        #return grasp_success <-- bool
        pass

    def handover_CV(self):
        # Function to detect handover location
        # returns if HO is signalled and position
        self.set_params()
        rospy.loginfo("Detecting HO Signal...")
        self.add_delay(time=4.0) # Delay to mimic detection time
        rospy.loginfo("Signal Detected. Starting Handover...")
        return True


    def object_CV(self):
        # Function to detect if the object is kept in the pickup area
        rospy.loginfo("Detecting object...")
        self.add_delay(time=4.0) # Delay to mimic detection time
        rospy.loginfo("Object detected!")
        return True


    def add_delay(self, time):
        self.set_params()
        rospy.sleep(time)
        return True


    def set_interaction_mode(self, switch):
        self.set_params()
        pass


    def calc_HO_loc(self):
        self.set_params()
        
        self.pos[0]+=self.del_x
        self.pos[1]+=self.del_y
        self.pos[2]+=self.del_z


    def is_satisfied(self):
        print("\n\nAre you satisfied with the Training?\n\n")
        # answer = yes or no
        answer = False
        return answer
    