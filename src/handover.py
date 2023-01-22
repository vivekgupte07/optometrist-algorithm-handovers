#! /usr/bin/env python

# Class of functions that generate motion to perform Handovers
import math
import rospy
import numpy as np
from gripper import gripper
from intera_interface import Limb
from geometry_msgs.msg import Pose, PoseStamped
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface.utility_functions import int2bool
from intera_core_msgs.msg import EndpointState, InteractionControlCommand
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions,
    InteractionPublisher
)



class Handover(object):

    def __init__(self):
        self.pos = [0, 0, 0]
        self.orientation = [0, 0, 0, 0]
        self.success = bool

        self.tip_name = 'right_hand'
        self.endpoint_name = 'right_hand'

        self.name = 'HOME'
        self.max_lin_accl = 6.0 
        self.max_rot_speed = 6.28
        self.max_rot_accl = 6.28
        self.speed_ratio = 1.0

        self.HO_detection_flag = 0
        self.past_force_x = 0
        self.past_force_y = 0
        self.past_force_z = 0

        self.set_params()
        self.set_interaction_params()
        
        self.forces = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.force_callback)

    
    def get_params(self):
        # Learning curriculum
        pass

    
    def set_params(self):
        # get_params()
        # Get new values for the params
        if self.name in ['OBSERVE', 'HOME', 'PICKUP']:
            self.max_lin_speed = 0.7 # Speed during other phases
        else:
            self.max_lin_speed = 0.4 # Speed during HO Reach


        self.delay = 2.0

        # Offests from CV generated human position
        self.del_x = -0.3
        self.del_y = -0.1
        self.del_z = 0.2

        self.limb = Limb()
        self.traj_options = TrajectoryOptions()
        self.poseStamped = PoseStamped()
        

    def set_interaction_params(self):
        self.K_impedance = [900.0, 900.0, 900.0, 50.0, 50.0, 50.0] 
        # impedance : [lin_x, lin_y, lin_z, rot_x, rot_y, rot_z]
        self.max_impedance = [0, 0, 0, 1, 1, 1]
        # max_imp : 1 --> Set max impedance
        #           0 --> Do not set max impedance
        self.interaction_control_mode = [1, 1, 1, 1, 1, 1]
        # A list of desired interaction control mode
        # (1: impedance 2: force 3: impedance with force limit 
        # 4: force with motion limit), one for each of the 6 directions")
        self.interact_frame = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        # Specify the reference frame for the interaction controller (default: [0, 0, 0, 1, 0, 0, 0]) 
        # [x_pos, y_pos, z_pos, w_q, x_q, y_q, z_q] (normalized values)
        self.in_endpoint_frame = False
        # Set desired reference frame to end point frame -- default: base_frame
        self.force_command = [0, 0, 0, 0, 0, 0] 
        # Force commands for each frame
        self.K_nullspace = [5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0] 
        # A list of desired nullspace stiffnesses (Nm/rad)
        self.disable_damping_in_force_control = False
        self.disable_reference_resetting = False
        # The reference signal is reset to actual position to avoid jerks/jumps when interaction 
        # parameters are changed (Set True to disable this function)
        self.rotations_for_constrained_zeroG = False
        # Allow arbitrary rotational displacements from the current orientation for constrained zero-G
        # Works only with a stationary reference orientation
        self.rate = 0
        # Desired publish rate for updating control commands (Default=10Hz)
        # Rate more than zero for interactive mode until Ctrl-C
        self.interaction_options = InteractionOptions()
        self.interaction_frame = Pose()
    

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
                self.success = False
                return False

            if result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
                self.success = True
                return True
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
                self.success = False
                return False
                
        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
            self.success = False
            return False


    def stop_trajectory(self):
        
        pass


    def set_positions(self, name):
        self.name = name
        if self.name == 'OBSERVE':
            self.pos = [0.8, 0.2, 0.2]
            self.orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to %s', name)
            self.go_to()

        elif self.name == 'HOME':
            self.pos = [0.5, 0.0, 0.2]
            self.orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to %s', name)
            self.go_to()

        elif self.name == 'PICKUP':
            self.pos = [0.8, 0.2, -0.03]
            self.orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to %s', name)
            self.go_to()

        elif self.name == 'HANDOVER' :
            # Here, get pos from Computer vision
            # is_person, pos = handover_CV()
            # if pos is not None:.... continue else go back to HOME
            # self.calc_HO_loc()
            self.pos = [0.95, 0.1, 0.2]
            self.orientation = [0.0739582150969, 0.907450368469, -0.127978429887, 0.39330081702]
            rospy.loginfo('Set next pose to %s', name)
            if self.go_to():
                self.name='TRANSFER'

        else:
            rospy.logerr('Wrong Position Name, setting next pose to HOME')
            self.pos = [0.4, 0.0, 0.2]
            self.orientation = [0, 0, 1, 0]
            self.max_speed = 0.1
            rospy.loginfo('Set next pose to HOME')
            self.go_to()

    
    def HO_flag(self):
        return self.HO_detection_flag


    def force_callback(self, msg):
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        print(math.sqrt(force_x**2+force_y**2+force_z**2))

        self.HO_detection_flag = 0

        if self.name == 'TRANSFER':
            if math.sqrt(force_x**2+force_y**2+force_z**2)>11.5 or math.sqrt(force_x**2+force_y**2+force_z**2)<6:
                self.HO_detection_flag = 1 

    def old_force_callback(self, msg):
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        print(math.sqrt(force_x**2+force_y**2+force_z**2))
        self.HO_detection_flag = 0
        if self.name=='TRANSFER':    
            if force_x>-7.00 or force_x<-20.00:
                print('x:', msg.wrench.force.x)
                self.HO_detection_flag = 1


            if force_y>3.50 or force_y<-2.00:
                print('y:', msg.wrench.force.y)
                self.HO_detection_flag = 1

            if force_z>(1.0) or force_z<(-8.50):   
                print('z:', msg.wrench.force.z)
                self.HO_detection_flag = 1

        self.past_force_x = force_x
        self.past_force_y = force_y
        self.past_force_z = force_z

    
    def add_timeout(self, duration=5.0):
        try:
            if not duration < 0:
                start_time = rospy.get_rostime().secs

                while (rospy.get_rostime().secs - start_time) <= duration:
                    if self.name=='TRANSFER':
                        if self.HO_flag():
                            break
                if self.name is not 'TRANSFER':
                    rospy.loginfo('Transfer Timed out')
                return True
            else:
                rospy.logerr('A negative duration makes no sense!')
        except rospy.ROSInterruptException:
            rospy.loginfo('Keyboard interrupt detected from the user. Exiting before trajectory completion.')

    
    def grasp(self, act):
        
        self.set_params()
        gripper(act)


    def grasp_detection(self):
        
        #check_grasp_detection()
        #return grasp_success <-- bool
        pass

    
    def handover_CV(self):
        
        # Function to detect handover location
        # returns if HO is signalled and position
        self.set_params()
        rospy.loginfo("Detecting HO Signal...")
        self.add_delay(time=2.0) # Delay to mimic detection time
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


    def interaction_mode(self, switch=False):
        
        self.set_params()
        self.set_interaction_params()
        # set the interaction control options in the current configuration

        self.interaction_options.set_interaction_control_active(switch) # (0) for deactivate
        self.interaction_options.set_K_impedance(self.K_impedance)
        self.interaction_options.set_max_impedance(int2bool(self.max_impedance))
        self.interaction_options.set_interaction_control_mode(self.interaction_control_mode)
        self.interaction_options.set_in_endpoint_frame(self.in_endpoint_frame)
        self.interaction_options.set_force_command(self.force_command)
        self.interaction_options.set_K_nullspace(self.K_nullspace)
            
        self.interaction_frame.position.x = self.interact_frame[0]
        self.interaction_frame.position.y = self.interact_frame[1]
        self.interaction_frame.position.z = self.interact_frame[2]
        self.interaction_frame.orientation.w = self.interact_frame[3]
        self.interaction_frame.orientation.x = self.interact_frame[4]
        self.interaction_frame.orientation.y = self.interact_frame[5]
        self.interaction_frame.orientation.z = self.interact_frame[6]
        self.interaction_options.set_interaction_frame(self.interaction_frame)
        
        self.interaction_options.set_disable_damping_in_force_control(self.disable_damping_in_force_control)
        self.interaction_options.set_disable_reference_resetting(self.disable_reference_resetting)
        self.interaction_options.set_rotations_for_constrained_zeroG(self.rotations_for_constrained_zeroG)

        # print the resultant interaction options once
        #rospy.loginfo(self.interaction_options.to_msg())
        
        if self.name=='HANDOVER':
            ic_pub = InteractionPublisher()
            
            if self.rate != 0:
                rospy.on_shutdown(ic_pub.send_position_mode_cmd)
            ic_pub.send_command(self.interaction_options, self.rate)
            if self.rate == 0:
               rospy.sleep(0.5)


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
    