#! /usr/env/bin python

'''
Experiment Script

'''
import os
import rospy
import perform_handover
from handover_class import Handover
from tune_parameters import Tuning_algo
# Start Experiment
# Ask for the user name (Deal with new and old user)
# go to Profile script
ta = Tuning_algo()
ta.create_profile()


# A block to choose whether to skip training (helpful for resuming an expt)

# Perform three demonstrations
for i in range(3):
	perform_handover()


# Check if the participant is ready
# Pause code till experimenter presses a button


# Begin tuning

ta.set_phase(phase='position_x')

ta.set_phase(phase='position_y')

ta.set_phase(phase='position_z')

ta.set_phase(phase='vel')

ta.set_phase(phase='force_th')

ta.set_phase(phase='delay')



# Evaluation 

# Perform 10 Handovers
for i in range(10):
	perform_handover()

# Think of more evaluation criteria...