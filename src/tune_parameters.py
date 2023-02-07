#! /usr/bin/env python

import os
import numpy as np
import random
import math
import rospy
from std_msgs.msg import String
import perform_handover
from numpy import loadtxt, savetxt
from getkey import getkey

class Tuning_algo(object):


	def __init__(self):
		
		self.phase = str
		self.choice = bool
		self.option = float
		self.final_choice = float

		self.pos_x = float
		self.pos_y = float
		self.pos_z = float
		self.vel   = float
		self.force_th = float
		self.delay = float
		
		self.low = float
		self.high = float
		self.break_th = float
		self.params = [0,0,0,0,0,0]
		self.satisfied = False
		self.name = str

	def set_phase(self, phase='position_x'):
		self.phase = phase
		self.set_initial_params()
		self.set_params()
		self.tuning()


	
	def tuning(self):
		print(self.low, self.high)

		mid = (self.low + self.high)/2		
		
		option_1 = mid

		# Randomly choosing between A or B for option 2
		option_2 = self.low 
		current_dirc = -1 
		x = -1
		noof_option_1 = 0

		######## Main tuning loop ########
		while not self.satisfied:
			try:
				# Calculating new step size
				current_step_size = abs(option_1-option_2)*(1-math.exp(x))
				print('current_step_size=%s' % current_step_size)
				
				self.option = option_1
				# Show first trajectory
				self.send_traj()

				self.option = option_2
				# Show second trajectory
				self.send_traj()

				## Get human response
				self.choose()

				
				## Take a step, calculate parameters for the next step
				if self.choice:
					# Flip the current direction
					current_dirc = -1 * current_dirc
					# Get new options
					#temp = option_2
					option_1 = option_2
					option_2 = option_1 + current_dirc * current_step_size
					x += 0.1
					noof_option_1 = 0

				else:
					current_dirc = -1 * current_dirc
					# Get new options
					option_2 = option_1 + current_dirc * current_step_size
					if noof_option_1>0:
						x+=0.1
					else:
						noof_option_1 += 1


				
				if option_1 > option_2:
					current_dirc = -1
				else:
					current_dirc = 1

				# Check for break criteria
				if (abs(option_1-option_2) <= self.break_th) or x >= -0.01:
				#if count > 3:					
					print('option_1=%s' % option_1)
					final_choice = option_1
					print('final_choice=%s' %final_choice)
					break
			except rospy.ROSInterruptException:
				rospy.logerr('Keyboard interruption detected.')
		self.option = final_choice
		self.store_result()
		return True


	def choose(self):
		while True:
			key = getkey()
			if key == '2':
				self.choice = True
				break
			elif key == '1':
				self.choice = False
				break
			else:
				rospy.loginfo('Wrong Key! Try again...\n')
				continue
		return True

	def set_initial_params(self):
		params = loadtxt('%s.csv' % self.name)
		
		if self.phase=='position_x':
			self.high = 1.0
			self.low = 0.5
			self.break_th = 0.05
		elif self.phase=='position_y':
			self.break_th = 0.05
			self.high = 0.2
			self.low = -0.2

		elif self.phase=='position_z':
			self.break_th = 0.05
			self.high = 0.25
			self.low = -0.1 
			
		elif self.phase=='velocity':
			self.break_th = 0.1
			self.high = 0.8
			self.low = 0.1

		elif self.phase=='force_th':
			self.break_th = 0.1
			self.high = 2.0
			self.low = 1.25

		elif self.phase=='delay':
			self.break_th=0.3
			self.high = 2.0
			self.low = 0.2



	def load_last_values(self):
		# Load the existing file of parameters
		params = loadtxt('%s.csv' % self.name)
		
		# Distribute the values in correct data structures
		self.pos_x = params[0]
		self.pos_y = params[1]
		self.pos_z = params[2]
		self.vel   = params[3]
		self.force_th = params[4]
		self.delay = params[5]


	def set_params(self):
		# Method to change a particular parameter during tuning
		self.load_last_values()

		if self.phase=='position_x':
			self.params = [self.option, self.pos_y, self.pos_z, self.vel, self.force_th, self.delay]
		
		elif self.phase=='position_y':
			self.params = [self.pos_x, self.option, self.pos_z, self.vel, self.force_th, self.delay]

		elif self.phase=='position_z':
			self.params = [self.pos_x, self.pos_y, self.option, self.vel, self.force_th, self.delay]

		elif self.phase=='velocity':
			self.params = [self.pos_x, self.pos_y, self.pos_z, self.option, self.force_th, self.delay]

		elif self.phase=='force_th':
			self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.option, self.delay]
		
		elif self.phase=='delay':
			self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.force_th, self.option]
		else:
			rospy.logerr('Wrong Phase!')


	def save_params(self):
		self.set_params()
		#print('%s' % self.set_params)
		savetxt('%s.csv' % self.name, np.array(self.params))


	def send_traj(self):
		self.save_params()
		print(self.option)
		perform_handover.main()


	def create_std_params(self, name):	
		self.pos_x = 0.9
		self.pos_y = 0.0
		self.pos_z = 0.2
		self.vel   = 0.4
		self.force_th = 1.35
		self.delay = 0.3
		
		std_param = np.array([self.pos_x, self.pos_y, self.pos_z, self.vel, self.force_th, self.delay])
		print('%s' %std_param)
		savetxt('%s.csv' %self.name , std_param)
	
	def create_profile(self):
		self.name = raw_input("Participant ID: ")
		### TO ADD: Check if this ID already exists and resume training / evaluate if it does

		# Create std params file
		self.create_std_params(self.name)

		rospy.init_node('tuning_algo')

		pub = rospy.Publisher('filename', String, latch=True)
		pub.publish(self.name)

		

	def store_result(self):
		self.save_params()
		
		# Move the created final file post tuning if needed

		pass
