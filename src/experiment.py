#! /usr/bin/env python

'''
Experiment Script

'''

import os
import rospy
from getkey import getkey
import perform_handover
from handover_class import Handover
from tune_parameters import Tuning_algo
# Start Experiment
ta = Tuning_algo()
ta.create_profile()
# A block to choose whether to skip training (helpful for resuming an expt)
resume = True
# Perform three demonstrations
for i in range(3):
	if not resume:
		perform_handover.main()
		pass

# Check if the participant is ready
# Pause code till experimenter presses a button
print("Press any key to continue! (or press ESC to quit!)")
if getkey() == chr(0x1b):
	rospy.logerr('Interruption called')
	exit()

# Begin tuning
rospy.loginfo("Begin tuning now...")

ta.start_tuning()



# Evaluation 

# Perform 10 Handovers
for i in range(10):
	perform_handover.main()

# Think of more evaluation criteria...