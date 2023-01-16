#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (InteractionOptions, InteractionPublisher)
from intera_motion_interface.utility_functions import int2bool

def main():
    rospy.init_node('set_interaction_options')

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

    rate = 10
    # Desired publish rate for updating control commands (Default=10Hz)


    # set the interaction control options in the current configuration
    interaction_options = InteractionOptions()

    interaction_options.set_interaction_control_active(int2bool(1)) # (0) for deactivate
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


if __name__ == '__main__':
    main()
