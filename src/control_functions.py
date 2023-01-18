#! /usr/bin/env python

# Main control loop for Sawyer Handover
# 

import rospy
from handover import Handover
from intera_core_msgs.msg import EndpointState

def main():
    
    rospy.init_node('control_functions_py')
    forces = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, force_callback)

    handover = Handover()

    # Get CV --> ID
    # Initializing
    satisfied = False
    counter = 0
    try:
        while not satisfied:
            
            handover.set_positions(name='OBSERVE')
            
            obj = False
            while not obj:
                obj = handover.object_CV()

            handover.add_delay(time=2.0) # Delay to avoid moving too soon
            
            handover.set_positions(name='PICKUP')
            handover.add_delay(time=2.0)

            handover.grasp(act='on')

            handover.add_delay(time=2.0) # Delay to allow grasping 

            handover.set_positions(name='HOME')

            if counter > 9:
                satisfied = handover.is_satisfied()
            if satisfied:
                break

            person = False
            while not person:
                person = handover.handover_CV()

            handover.add_delay(time=2.0) # Delay before starting HO (Param)

            handover.set_positions("HANDOVER") # Performing Reach Phase
            handover.add_delay(time=2.0)

            handover.set_interaction_mode(switch='on')

            handover.set_interaction_mode(switch='off')

            counter += 1

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


def force_callback(msg):
    pass


if __name__ == '__main__':
    main()
