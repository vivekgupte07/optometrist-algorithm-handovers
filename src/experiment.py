#! /usr/bin/env python

'''
Experiment Script

'''

import os
import rospy
import numpy as np
import perform_handover
from numpy import savetxt
from getkey import getkey
from handover_class import Handover
from tune_parameters import Tuning_algo

# Start Experiment
ta = Tuning_algo()
profile_status = ta.create_profile()
start=rospy.get_rostime().secs

# A block to choose whether to skip training (helpful for resuming an expt)
resume = False
eval_resume = False
if resume:
	j= raw_input('Enter number of interventions:\n')
# Perform three demonstrations
for i in range(5):
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

#ta.start_tuning()

# Check if the participant is ready
# Pause code till experimenter presses a button
print("Press any key to continue! (or press ESC to quit!)")
if getkey() == chr(0x1b):
	rospy.logerr('Interruption called')
	exit()

# Evaluation 
eval_start = rospy.get_rostime().secs
# Perform 5 Handovers
for i in range(5):
	print('Handover number: %s' %(i+1))
	perform_handover.main()

#Evaluations
print("Press any key to continue! (or press ESC to quit!)")
if getkey() == chr(0x1b):
	rospy.logerr('Interruption called')
	exit()

ta.evaluations()



end = rospy.get_rostime().secs

eval_time = end - eval_start
total_task_time = end - start

data_to_export = [eval_time, total_task_time]
data_to_export = np.array(data_to_export)

number = raw_input("Enter Participant ID for the last time: \n")
savetxt('/home/miniproj/catkin_ws/src/vivek-handovers/telemetry/%s_total.csv' %number, data_to_export)
