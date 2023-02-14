#! /usr/bin/env python

# Class of functions that generate motion to perform Handovers
import os
import math
import rospy
import numpy as np
from gripper import gripper
from std_msgs.msg import String
import matplotlib.pyplot as plt
from numpy import savetxt, loadtxt
from intera_interface import Limb
from tune_parameters import Tuning_algo
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
        self.timeout = bool
        
        self.tip_name = 'right_hand'
        
        self.eq_force_array = np.zeros(50)
        self.pubcounter = 0
        self.name = 'HOME'
        self.max_lin_accl = 15.0
        self.max_rot_speed = 20.00
        self.max_rot_accl = 20.00
        self.speed_ratio = 1.0
        self.moving_avg = list()
        self.HO_detection_flag = 0
        self.past_force = -1
        self.past_force_to_save = -1
        self.force_activation_counter = 0
        self.force_avg = 0
        self.filename = str
        self.dir = '/home/miniproj/catkin_ws/src/vivek-handovers/src/profiles'
        #rospy.init_node("handover_class_py")
        #sub= rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.vel_callback)
        sub = rospy.Subscriber('/filename', String, self.filename_callback)
        self.eq_force_to_save = 0
        self.log_data = True
        self.log = [[-9, -9, -9, 0],]
        self.vel_log = [0,]

        rospy.sleep(0.5)
        self.set_params()
        self.set_trajectory_params()
        self.set_interaction_params()


    def filename_callback(self, msg):
        self.filename = msg.data
        #print(self.filename_)


    def get_params(self):
        params = loadtxt(os.path.join(self.dir, '%s.csv' % self.filename)) #[0.8, 0.0, 0.2, 0.4, 1.55, 0.4, 1] #
        return params
    
    def set_params(self):
        
        params_=self.get_params()

        # Get new values for the params from the saved file   
        if self.name == 'TRANSFER':
            self.delay = params_[5]
        else:
            self.delay = 2.0
        self.low_force_factor = 0
        self.high_force_factor = params_[4]
        self.past_force = -1
        

    def set_trajectory_params(self):
        traj_params = self.get_params()  
        if self.name in ['OBSERVE', 'HOME', 'PICKUP']:
            self.max_lin_speed = 0.4 # Speed during other phases
        else:
            self.max_lin_speed = traj_params[3]     # Speed during HO Reach

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
    

    def set_positions(self, name):

        self.name = name
        if self.name == 'OBSERVE':
            self.pos = [0.73, 0.0, 0.2]
            self.orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to %s', name)
            self.go_to()

        elif self.name == 'HOME':
            self.pos = [0.5, 0.0, 0.2]
            self.orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to %s', name)
            self.go_to()

        elif self.name == 'PICKUP':
            self.pos = [0.73, 0.0, 0.0]
            self.orientation = [0, 1, 0, 0]
            rospy.loginfo('Set next pose to %s', name)
            self.go_to()

        elif self.name == 'HANDOVER' :
            if False:#self.log_data:
                x = np.random.uniform(low=0.85, high=0.95)
                y = np.random.uniform(low=-0.20, high=0.20)
                z = np.random.uniform(low=0.15, high=0.25)
                self.pos = [x, y, z]
            else:
                pos_params = self.get_params()
                self.pos = [pos_params[0], pos_params[1], pos_params[2]]

            self.HO_pos = self.pos

            # self.orientation = [0.0739582150969, 0.907450368469, -0.127978429887, 0.39330081702]
            self.orientation = [-0.00715778427529, 0.97663386697, 0.0170727236209, 0.21411113494]
            rospy.loginfo('Set next pose to %s', name)

            if self.go_to():
                self.forces = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.force_callback)
                self.name='TRANSFER'
                self.get_params()
            
        else:
            rospy.loginfo('Wrong Position Name, setting next pose to HOME')
            self.pos = [0.4, 0.0, 0.2]
            self.orientation = [0, 0, 1, 0]
            self.max_speed = 0.1
            rospy.loginfo('Set next pose to HOME')
            self.go_to()

    
    def go_to(self): 
        
        try:
            self.set_params()
            self.set_trajectory_params()
                        
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
            exit()
            return False


    def stop_trajectory(self):
        
        pass

    def grasp(self, act):
        try:
            gripper(act)

            if act=='open':
                params=self.get_params()
                self.add_delay(params[5])
        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
            exit()


    def grasp_detection(self):
        #check_grasp_detection()
        #return grasp_success
        pass


    def force_callback(self, msg):
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        eq_force = math.sqrt(force_x**2+force_y**2+force_z**2)

        self.HO_detection_flag = 0

        avg = 0
        for i in range(len(self.moving_avg)):
            avg += self.moving_avg[i]
        if len(self.moving_avg)>0:
            avg = avg/len(self.moving_avg)


        if self.pubcounter==5:
            self.force_avg=avg+3
            print(self.force_avg)


        if self.name == 'TRANSFER':
            #print(self.pubcounter, eq_force)    
            if self.pubcounter>5:
                if avg > self.high_force_factor * self.force_avg:
                    self.HO_detection_flag=1
            if self.HO_detection_flag!=1:
                #print(eq_force, avg, self.force_avg)
                pass
        
            self.pubcounter += 1
            
        if self.pubcounter >= 5 and len(self.moving_avg)<5:
                self.moving_avg.append(eq_force)
        if len(self.moving_avg) >= 5:
            for i in range(len(self.moving_avg)-1):
                self.moving_avg[i]=self.moving_avg[i+1]
            self.moving_avg[-1]=eq_force

            
    def HO_flag(self):
        return self.HO_detection_flag

    
    def add_timeout(self, duration=5.0):
        try:
            if self.name=='TRANSFER':
                start_time = rospy.get_rostime().secs
                self.timeout = True
                while (rospy.get_rostime().secs - start_time) <= duration:    
                    if self.HO_flag():
                        self.timeout = False
                        break
                    else:
                        self.timeout = True
                return True
        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
            exit()

    
    def get_timeout_state(self):
        return self.timeout

    
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
        self.add_delay(time=2.0) # Delay to mimic detection time
        rospy.loginfo("Object detected!")
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


    def vel_callback(self, msg):
        vel_x = msg.twist.linear.x
        vel_y = msg.twist.linear.y
        vel_z = msg.twist.linear.z
        eq_vel= math.sqrt(vel_x**2+vel_y**2+vel_z**2)
        if self.name=='HANDOVER':
            #print(eq_vel)
            self.vel_log.append(eq_vel)

        if self.name=='TRANSFER':
            #np.savetxt('vel_log.csv', self.vel_log)
            pass

    def add_delay(self, time):
        
        self.set_params()
        rospy.sleep(time)
        return True
    

    def logger(self):
        new_entry = self.pos + [self.eq_force_to_save, self.past_force_to_save]
        return True


    def save_log(self):
        
        if self.logger():
            print("Saving....")
            np.savetxt('log_forces.csv', np.array(self.log))
