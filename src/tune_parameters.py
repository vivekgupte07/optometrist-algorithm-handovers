#! /usr/bin/env python

import os
import numpy as np
import random
import rospy
import perform_handover
from numpy import loadtxt, savetxt
from getkey import getkey, keys

class Tuning_algo(object):


	def __init__(self):
		
		self.phase = str
		self.choice = bool
		
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


	def set_phase(self, phase='position_x'):
		self.phase = phase
		self.tuning()


	
	def tuning(self):
		
		mid = (self.low + self.high)/2		
		
		option_1 = mid

		# Randomly choosing between A or B for option 2
		option_2 = self.low if random.uniform(0,1) > 0.5 else self.high 
		
		current_dirc = -1 if option_1==self.low else 1
		x = -1

		######## Main tuning loop ########
		while not self.satisfied():
			try:
				# Calculating new step size
				current_step_size = abs(option_1-option_2)*(1-exp(x))
				
				
				# Incrementing x after calculating step size for next iteration
				if x < 0:
					x += 0.05
				else:
					x = 0

				# Setting new current direction
				if option_1 > option_2:
					current_dirc = -1
				elif option_1 < option_2:
					current_dirc = 1
				else:
					break
				
				self.current_option = 'A'
				# Show first trajectory
				self.send_traj()

				self.current_option = 'B'
				# Show second trajectory
				self.send_traj()

				## Get human response
				self.choose()

				
				## Take a step, calculate parameters for the next step
				if self.choice:
					# Flip the current direction
					current_dirc = -1 * current_dirc

					# Get new options
					temp = option_2
					option_2 = option_1 + current_dirc * current_step_size
					option_1 = temp

				else:
					# Get new options
					option_2 = option_1 + current_dirc * current_step_size

				## Check for break criteria
				if (abs(option_1-option_2) <= self.break_th):
					final_choice = option_1
					break
			except rospy.ROSInterruptionException:
				rospy.logerr('Keyboard interruption detected.')

		self.final_choice = final_choice
		self.store_result()
		return True


	def choose(self):
		while True:
			key = getkey()
			if key == '1':
				self.choice = True
				break
			elif key == '2':
				self.choice = False
				break
			else:
				rospy.loginfo('Wrong Key! Try again...\n')
				continue
		return True

	
	def load_last_values(self):
		# Load the existing file of parameters
		params = loadtxt('params.csv')
		self.pos_x = params[0]
		self.pos_y = params[1]
		self.pos_z = params[2]
		self.vel   = params[3]
		self.force_th = params[4]
		self.delay = params[5]
		# Distribute the values in correct data structures		


	def set_params(self):
		self.load_last_values()
		option = self.current_option
		
		# Method to change a particular parameter during tuning
		if self.phase=='position_x':
			self.break_th = 0.05
			self.high = 1.0
			self.low = 0.5

			if option=='A':
				self.params = [self.option_1, self.pos_y, self.pos_z, self.vel, self.force_th, self.delay] 
			if option=='B':
				self.params = [self.option_2, self.pos_y, self.pos_z, self.vel, self.force_th, self.delay]
		
		if self.phase=='position_y':
			self.break_th = 0.05
			self.high = 0.2
			self.low = -0.2
			if option=='A':
				self.params = [self.pos_x, self.option_1, self.pos_z, self.vel, self.force_th, self.delay]
			if option=='B':
				self.params = [self.pos_x, self.option_2, self.pos_z, self.vel, self.force_th, self.delay]

		if self.phase=='position_z':
			self.break_th = 0.05
			self.high = 0.25
			self.low = -0.1
			if option=='A':
				self.params = [self.pos_x, self.pos_y, self.option_1, self.vel, self.force_th, self.delay]
			if option=='B':
				self.params = [self.pos_x, self.pos_y, self.option_2, self.vel, self.force_th, self.delay]

		if self.phase=='velocity':
			self.break_th = 0.1
			self.high = 2.0
			self.low = 0.1
			if option=='A':
				self.params = [self.pos_x, self.pos_y, self.pos_z, self.option_1, self.force_th, self.delay]
			if option=='B':
				self.params = [self.pos_x, self.pos_y, self.pos_z, self.option_2, self.force_th, self.delay]

		if self.phase=='force_th':
			self.break_th = 0.05
			self.high = 1.0
			self.low = 0.5
			if option=='A':
				self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.option_1, self.delay]
			if option=='B':
				self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.option_2, self.delay]
		
		if self.phase=='delay':
			self.break_th = 0.1
			self.high = 1.0
			self.low = 0.1
			if option=='A':
				self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.force_th, self.option_1]
			if option=='B':
				self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.force_th, self.option_2]
		else:
			rospy.logerr('Wrong Phase!')


	def save_params(self):
		self.set_params()
		savetxt('%s.csv', self.name, np.array(self.params))


	def send_traj(self):
		self.save_params()
		perform_handover()

	
	def create_std_params(self):	
		
		self.pos_x = 0.8
		self.pos_y = 0.0
		self.pos_z = 0.2
		self.vel   = 0.4
		self.force_th = 1.35
		self.delay = 0.3
		
		std_param[0] = self.pos_x
		std_param[0] = self.pos_y
		std_param[0] = self.pos_z
		std_param[0] = self.vel
		std_param[0] = self.force_th
		std_param[0] = self.delay

		savetxt('%s.csv', self.name, np.array(std_param))

	
	def create_profile(self):
		
		self.name = rawinput("Participant ID: ")
		
		### TO ADD: Check if this ID already exists and resume training / evaluate if it does

		# Create std params file
		self.create_std_params()


	def store_result(self):
		self.current_option = A
		self.save_params()
		
		# Move the created final file post tuning if needed

		pass
