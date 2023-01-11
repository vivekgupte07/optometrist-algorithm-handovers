#! /usr/bin/env python

import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

def calculateIK(position, orientation, use_advanced_options = True):
    ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
    limb = "right"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                ),
                orientation=Quaternion(
                    x=0.0,
                    y=1.0,
                    z=0.0,
                    w=0.0,
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = [-0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        #The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, -0.3, -0.5]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(resp.joints[0].name, resp.joints[0].position)))
        #rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        #rospy.loginfo("Response Message:\n%s", resp)


    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False

    return resp.joints[0].position


def main():
			
	Kp = 10.0
	Ki = 0.0
	Kd = 1.0
	dt = 1.0/100.0
	# Control velocities and accelerations
	
	current_pos = [0, 0, 0]
	orientation = [0.0, 1.0, 0.0, 0.0]
	goal_pos = [0.8, -0.2, 0.2] # Goal Position
	# orientation = [0.0, 0.0, 1.0, 0.0] # Goal Orientation
	

	try:
		rospy.init_node('cartesian_movement_py')
		# Create necessary objects and add information to these objects
		
		# This is where we introduce next points (PID Controller)
		error = [0.2,0.2,0.2]
		past_err = [0, 0, 0]
		
		k = [0.0,0.0,0.0]
		next_pos = [0.0, 0.0, 0.0]

		counter = 0
		next_pos_list = []
		
		
		while abs(error[0])>0.01 or abs(error[1])>0.01 or abs(error[2]>0.01):
			
			limb = Limb()
			if counter == 0:
				traj = MotionTrajectory(limb = limb)
			wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.1,
                                         max_joint_accel=0.1)	
			waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

			tip_name = 'right_hand'
			joint_names = limb.joint_names()
			endpoint_state = limb.tip_state(tip_name)

			#joint_angles = limb.joint_ordered_angles()
			#waypoint.set_joint_angles(joint_angles = joint_angles)
			#traj.append_waypoint(waypoint.to_msg())
			
			#if counter == 0:
			current_pos[0] = endpoint_state.pose.position.x
			current_pos[1] = endpoint_state.pose.position.y
			current_pos[2] = endpoint_state.pose.position.z
	
			for i in range(len(current_pos)):
				error[i] = goal_pos[i]-current_pos[i]			
			
			for n in range(len(current_pos)):
				k[n] = Kp*error[n] + Ki*error[n]*dt + Kd*(error[n]-past_err[n])/dt
			
			past_err = error
			for m in range(len(k)):
				next_pos[m] = current_pos[m] + 0.1*k[m]	
				if next_pos[m] > 0.8:
					next_pos[m] = 0.8
			next_pos_list.append(next_pos)
			joint_angles = calculateIK(next_pos, orientation)

			# Set new joint angles
			if joint_angles:
				waypoint.set_joint_angles(joint_angles = joint_angles)
				traj.append_waypoint(waypoint.to_msg())
				rospy.loginfo(endpoint_state.pose)
				rospy.loginfo(waypoint.to_msg().pose)
				#current_pos = next_pos
				result = traj.send_trajectory(wait_for_result=False ,timeout=None)
				counter += 1
			else:
				result = None
				break
		 # None means inifinite timeout
		if result is None:
			rospy.logerr('Trajectory FAILED to send')
			return

		if result.result:
			rospy.loginfo('Motion controller successfully finished the trajectory!')
		else:
			rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
        
	except rospy.ROSInterruptException:
		print(next_pos_list)
		rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
