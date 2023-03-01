#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Pose
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (InteractionOptions, InteractionPublisher)
from intera_motion_interface.utility_functions import int2bool
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState

def get_params():
# [lin_speed, lin_accel, rot_speed, rot_accel, ratio]
    params = [0.3, 0.3, 1.57, 1.57, 1.0]
    return params


def set_params():
    # [lin_speed, lin_accel, rot_speed, rot_accel, ratio]
    params = get_params()
    max_linear_speed = params[0] 
    max_linear_accel = params[1] 
    max_rotational_speed = params[2]
    max_rotational_accel = params[3]
    max_joint_speed_ratio = params[4]

    return max_linear_speed, max_linear_accel, max_rotational_speed, max_rotational_accel, max_joint_speed_ratio

def force_callback(msg):
    if (msg.wrench.force.x >= 3.00 or msg.wrench.force.x >= 3.00 or msg.wrench.force.x >= 3.00):
        set_interactive_mode(mode=True)
    else:
        set_interactive_mode(mode=False)
    
def set_interactive_mode(mode):
    K_impedance = [500.0, 500.0, 500.0, 10.0, 10.0, 10.0] 
    # impedance : [lin_x, lin_y, lin_z, rot_x, rot_y, rot_z]
    max_impedance = [0, 0, 0, 0, 0, 0]
    # max_imp : 1 --> Set max impedance
    #           0 --> Do not set max impedance
    interaction_control_mode = [1, 1, 1, 1, 1, 1]
    # A list of desired interaction control mode
    # (1: impedance 2: force 3: impedance with force limit 
    # 4: force with motion limit), one for each of the 6 directions")
    interact_frame = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    # Specify the reference frame for the interaction controller (default: [0, 0, 0, 1, 0, 0, 0]) 
    # [x_pos, y_pos, z_pos, w_q, x_q, y_q, z_q] (normalized values)
    in_endpoint_frame = False
    # Set desired reference frame to end point frame -- default: base_frame
    endpoint_name = 'right_hand'
    force_command = [0, 0, 0, 0, 0, 0] 
    # Force commands for each frame
    K_nullspace = [5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0] 
    # A list of desired nullspace stiffnesses (Nm/rad)
    disable_damping_in_force_control = False
    disable_reference_resetting = False
    # The reference signal is reset to actual position to avoid jerks/jumps when interaction 
    # parameters are changed (Set True to disable this function)
    rotations_for_constrained_zeroG = False
    # Allow arbitrary rotational displacements from the current orientation for constrained zero-G
    # Works only with a stationary reference orientation
    rate = 0
    # Desired publish rate for updating control commands (Default=10Hz)
    # set the interaction control options in the current configuration
    interaction_options = InteractionOptions()
    interaction_options.set_interaction_control_active(mode) # (0) for deactivate
    interaction_options.set_K_impedance(K_impedance)
    interaction_options.set_max_impedance(int2bool(max_impedance))
    interaction_options.set_interaction_control_mode(interaction_control_mode)
    interaction_options.set_in_endpoint_frame(in_endpoint_frame)
    interaction_options.set_force_command(force_command)
    interaction_options.set_K_nullspace(K_nullspace)

    interaction_frame = Pose()
    interaction_frame.position.x = interact_frame[0]
    interaction_frame.position.y = interact_frame[1]
    interaction_frame.position.z = interact_frame[2]
    interaction_frame.orientation.w = interact_frame[3]
    interaction_frame.orientation.x = interact_frame[4]
    interaction_frame.orientation.y = interact_frame[5]
    interaction_frame.orientation.z = interact_frame[6]
    interaction_options.set_interaction_frame(interaction_frame)

    interaction_options.set_disable_damping_in_force_control(disable_damping_in_force_control)
    interaction_options.set_disable_reference_resetting(disable_reference_resetting)
    interaction_options.set_rotations_for_constrained_zeroG(rotations_for_constrained_zeroG)

    # print the resultant interaction options once
    rospy.loginfo(interaction_options.to_msg())

    ic_pub = InteractionPublisher()
    #rospy.sleep(0.5)
    if rate != 0:
        rospy.on_shutdown(ic_pub.send_position_mode_cmd)
    ic_pub.send_command(interaction_options, rate)
    if rate == 0:
       rospy.sleep(0.5)

def main():

    rospy.init_node('parameterized_cartesianpose_py')
    force = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, force_callback)
    print(force)
    retraction = True    
    try:
        while True:
            
            if retraction:
                position = [0.5, -0.0, 0.2]
                orientation = [0.0, 1.0, 0.0, 0.0]
                print("Retracting......")
                retraction = False

            else:
                position = [0.8, 0.0, 0.2]
                orientation = [-0.538174621635, 0.674156024416, -0.3945294507, 0.316588445622]
                print("Reaching....")
                retraction = True

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
            #rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=None)
            if result is None:
                rospy.logerr('Trajectory FAILED to send')
                return

            if result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
                rospy.sleep(4.0)
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                             result.errorId)
                break

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
