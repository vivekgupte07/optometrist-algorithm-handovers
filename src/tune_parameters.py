#! /usr/bin/env python

import numpy as np
import random
import rospy
import perform_handover

class Tuning_Algorithm(object):

	def __init__(self):
		
		self.final_choice = float
		self.way_out = bool

		#self.initial_vals = self.get_initial_values()
		
		self.low = 0
		self.high = 1
		
		self.mid = (self.high + self.low)/2
		self.break_th = 0.1

	def tuning(self):
		option_1 = mid
		option_2 = low if random.uniform(0,1) > 0.5 else high
		current_dirc = -1 if option_1==low else 1
		x = -1

		self.set_params(phase=phase, param=option_1, file=A)
		self.set_params(phase=phase, param=option_2, file=B)


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

				# Show first trajectory
				self.send_traj(option='A')
				# Show second trajectory
				self.send_traj(option='B')

				## Get human response
				resp = self.choose()
				
				# Evaluate the response
				if resp:
					choice = option_1
				else:
					choice = option_2

				## Take a step
				if choice == option_1:
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
				if (abs(option_1-option_2) <= self.break_th) or (self.way_out):
					final_choice = option_1
					break
			except rospy.ROSInterruptionException:
				rospy.logerr('Keyboard interruption detected.')

		self.final_choice = final_choice
		self.store_result()
		return True


	def choose(self):

		'''
		Function will ask user for input and wait for an answer
		'''
		
		pass

	
	def get_initial_values(self):
		
		'''
		Function will get flags according to the tuning phase
		Will use flags to return predetermined low and high values,
		as well as threshold values for break for every parameter being tuned
		'''
		
		pass

	def set_params(self, phase, option):

		if phase=='position_x'

			'''
			here, set other parameters to std and set the parameter in question to option
			'''		
			pass

		if phase=='position_y':
			pass

		if phase=='position_z':
			pass

		if phase=='velocity':
			pass

		if phase=='delays':
			pass
		'''
		add more phases here
		'''	
		self.send_params()


	def save_params(self, option)
		# save params to a file
		pass


	def send_traj(self, option):

		perform_handover(option)

	def store_result(self):

		'''
		Function will be called after tuning of one parameter and will store the final 
		value of the tuned parameter.
		'''
		pass
