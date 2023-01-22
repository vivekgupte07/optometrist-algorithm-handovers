#! /usr/bin/env python

# Main control loop for Sawyer Handover 

import rospy
from handover import Handover
from intera_core_msgs.msg import EndpointState

PHASE = 1

def main():
    rospy.init_node('control_functions_py')
    handover = Handover()
    # Get CV --> ID
    # initialize
    satisfied = False
    counter = 0
    try:
        while not satisfied:
            handover.set_positions(name='OBSERVE')

            obj = False
            while not obj:
                obj = handover.object_CV()
            
            handover.set_positions(name='PICKUP')
            
            # Grasping the object
            handover.add_delay(0.5)
            handover.grasp(act='close')
            handover.add_delay(time=0.50)

            handover.set_positions(name='HOME')

            person = False
            while not person:
                person = handover.handover_CV()

            handover.add_delay(time=0.5) # Delay before starting HO (Param)
            handover.set_positions("HANDOVER") # Performing Reach Phase
            handover.add_delay(0.5)

            # Handover period with a 5 second timeout
            handover.interaction_mode(False) #### Currently Turned off #### set 'True' to turn on #####
            
            while not handover.add_timeout(duration=5.0):
                handover.add_delay(0.01)
                if handover.HO_flag():
                    break

            handover.grasp(act='open')
            handover.add_delay(0.3)
            handover.interaction_mode(False)
            if counter > 9:
                satisfied = handover.is_satisfied()

            counter += 1

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')



if __name__ == '__main__':
    main()
 