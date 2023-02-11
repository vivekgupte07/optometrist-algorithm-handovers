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
		self.params = [0,0,0,0,0,0,1]
		self.satisfied = False
		self.name = str

		self.dir = '/home/miniproj/catkin_ws/src/vivek-handovers/src/profiles'

	
	def set_phase(self, phase='velocity'):
		self.phase = phase
		self.set_initial_params()
		self.set_params()
		self.tuning()


	def tuning(self):
		print(self.low, self.high)

		mid = (self.low + self.high)/2		
		noof_option_1 = 0
		step_size = self.break_th
		
		high=self.high
		low =self.low


		#Step 1, always the same.
		option_1 = self.low
		self.option = option_1
		self.send_traj()
		option_2 = self.high
		self.option = option_2
		self.send_traj()

		self.choose()

		if self.choice:
			low = self.low
			high = mid
			option_1 = low
			option_2 = high
			side = 'left'
		
		else:
			low = mid
			high = self.high
			option_1 = high
			option_2 = low
			side = 'right'

		i = 0

		######## Main tuning loop ########
		while not self.satisfied:
			try:
				if option_1 > option_2:
					dirc = -1
				else:
					dirc = 1
				
				self.option = option_1
				# Show first trajectory	
				self.send_traj()

				self.option = option_2
				# Show second trajectory
				self.send_traj()

				## Get human response
				self.choose()

				if (not self.choice and option_2==mid) or (self.choice and option_1==mid):
					i+=1
					if self.choice:
						noof_option_1+=1
					print('no of opt1:%s' %noof_option_1, side)
					if side=='left':
						low = mid 
						high = self.high - i*step_size
						option_1 = low
						option_2 = high
						side='right'
					
					else:
						high=mid
						low= self.low + i*step_size
						option_1 = high
						option_2 = low
						side='left'

				else:
					## Take a step, calculate parameters for the next step
					if self.choice:
						option_2 = option_2 - dirc*step_size

					else:
						temp = option_2
						option_2 = option_1 + dirc*step_size
						option_1 = temp


				# Check for break criteria
				if abs(option_1-option_2) < self.break_th  or noof_option_1==4:
				#if count > 3:					
					print('option_1=%s' % option_1)
					final_choice = option_1
					print('final_choice=%s' %final_choice)
					break
			except rospy.ROSInterruptException:
				rospy.logerr('Keyboard interruption detected.')
		self.option = final_choice
		self.save_params()
		return True


	def set_initial_params(self):
		params = loadtxt(os.path.join(self.dir, '%s.csv' % self.name))
		
		if self.phase=='velocity':
			self.phase_no =1
			self.break_th = 0.1
			self.high = 0.8
			self.low = 0.1

		elif self.phase=='position_x':
			self.phase_no = 2
			self.high = 1.0
			self.low = 0.80
			self.break_th = 0.05
		elif self.phase=='position_y':
			self.phase_no =3
			self.break_th = 0.05
			self.high = 0.2
			self.low = -0.2

		elif self.phase=='position_z':
			self.phase_no =4
			self.break_th = 0.05
			self.high = 0.25
			self.low = 0.15
			

		elif self.phase=='force_th':
			self.phase_no =5
			self.break_th = 0.1
			self.high = 2.0
			self.low = 1.25

		elif self.phase=='delay':
			self.phase_no =6
			self.break_th=0.3
			self.high = 2.0
			self.low = 0.2


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



	def start_tuning(self):
		self.resume_tuning()


	def resume_tuning(self):
		
		if self.params[6]==1:
			rospy.loginfo('Tuning velocity')
			self.set_phase(phase='velocity')
			self.params[6]=2
			self.save_params()
		
		if self.params[6]==2:
			rospy.loginfo('Tuning X')
			self.set_phase(phase='position_x')
			self.params[6]=3
			self.save_params()
		
		if self.params[6]==3:
			rospy.loginfo('Tuning Y')
			self.set_phase(phase='position_y')
			self.params=4
			self.save_params()

		if self.params[6]==4:
			rospy.loginfo('Tuning Z')
			self.set_phase(phase='position_z')
			self.params=5
			self.save_params()

		if self.params[6]==5:
			rospy.loginfo('Tuning force')
			self.set_phase(phase='force_th')
			self.params=6
			self.save_params()

		if self.params[6]==6:
			rospy.loginfo('Tuning delay')
			self.set_phase(phase='delay')
			self.save_params()

		return

	
	def load_last_values(self):
		# Load the existing file of parameters
		self.params = loadtxt(os.path.join(self.dir, '%s.csv' % self.name))
		print("Loading values from profile")
		# Distribute the values in correct data structures
		self.pos_x = self.params[0]
		self.pos_y = self.params[1]
		self.pos_z = self.params[2]
		self.vel   = self.params[3]
		self.force_th = self.params[4]
		self.delay = self.params[5]
		self.phase_no = self.params[6]


	def set_params(self):
		# Method to change a particular parameter during tuning
		self.load_last_values()

		if self.phase=='position_x':
			self.params = [self.option, self.pos_y, self.pos_z, self.vel, self.force_th, self.delay, self.phase_no]
		
		elif self.phase=='position_y':
			self.params = [self.pos_x, self.option, self.pos_z, self.vel, self.force_th, self.delay, self.phase_no]

		elif self.phase=='position_z':
			self.params = [self.pos_x, self.pos_y, self.option, self.vel, self.force_th, self.delay, self.phase_no]

		elif self.phase=='velocity':
			self.params = [self.pos_x, self.pos_y, self.pos_z, self.option, self.force_th, self.delay, self.phase_no]

		elif self.phase=='force_th':
			self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.option, self.delay, self.phase_no]
		
		elif self.phase=='delay':
			self.params = [self.pos_x, self.pos_y, self.pos_z, self.vel, self.force_th, self.option, self.phase_no]
		else:
			rospy.logerr('Wrong Phase!')


	def save_params(self):
		self.set_params()
		savetxt(os.path.join(self.dir, "%s.csv" %self.name), np.array(self.params))


	def send_traj(self):
		self.save_params()
		print('Current option value: %s' %self.option)
		perform_handover.main()


	def create_std_params(self, name):	
		self.pos_x = 0.8
		self.pos_y = 0.0
		self.pos_z = 0.2
		self.vel   = 0.4
		self.force_th = 1.35
		self.delay = 0.3
		self.phase_no = 1
		path = os.path.join(self.dir, self.name + '.' + 'csv')
		if os.path.exists(path):
			print('Already exists!')
			self.load_last_values()

		else:
			self.params = np.array([self.pos_x, self.pos_y, self.pos_z, self.vel, self.force_th, self.delay, self.phase_no])
			savetxt(path, self.params)
		print(self.params)
	
	def create_profile(self):
		self.name = raw_input("Participant ID: ")

		# Create std params file
		self.create_std_params(self.name)

		rospy.init_node('tuning_algo')

		pub = rospy.Publisher('filename', String, latch=True)
		pub.publish(self.name)
