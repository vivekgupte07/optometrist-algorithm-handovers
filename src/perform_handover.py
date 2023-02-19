#! /usr/bin/env python

# Main control loop for Sawyer Handover 
import os
import rospy
from playsound import playsound
from handover_class import Handover

def main(audio=False, option=True):
    handover = Handover()
    audio_dir = '/home/miniproj/catkin_ws/src/vivek-handovers/audio'
    # initialize
    counter = 0
    timeout = False
    timeout = handover.get_timeout_state()
     
    try:
        if not timeout:
            handover.set_positions(name='OBSERVE')
            
            obj = False
            while not obj:
                obj = handover.object_CV()
                
            handover.set_positions(name='PICKUP')
            
            # Grasping the object
            handover.add_delay(0.50)
            handover.grasp(act='close')
            handover.add_delay(0.50)
            
            handover.set_positions(name='HOME')
            
        else:
            handover.set_positions(name='HOME')
        
        if audio:
                if option:
                    playsound(os.path.join(audio_dir, 'option1.' + 'mp3'))
                else:
                    playsound(os.path.join(audio_dir, 'option2.' + 'mp3'))
           
        person = False
        while not person:
            person = handover.handover_CV()

        
        handover.add_delay(0.5) # Delay before starting HO (Param)
        handover.set_positions(name='HANDOVER') # Performing Reach Phase
        handover.add_delay(0.0)


        # Handover period with a 5 second timeout
        handover.interaction_mode(False) #### Currently Turned off #### set 'True' to turn on #####
        
        while not handover.add_timeout(duration=15.0):
            pass

        timeout = handover.get_timeout_state()
        if not timeout:
            handover.grasp(act='open')
            handover.save_log()

        handover.add_delay(0.40)
        handover.interaction_mode(False)
        handover.set_positions(name='OBSERVE')

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')
        exit()



if __name__ == '__main__':
    main()
 