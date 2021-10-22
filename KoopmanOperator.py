"""
This is a code to calculate possibe Koopman Operator from a known database
"""

import numpy as np
import pandas as pd
from numpy import cos, sin
from matplotlib import pyplot as plt

class KoopmanOperator:
	def __init__(self, _initmeasure, _dt):
	
		# define size of Koopman Operator
		self.basissize = 57
		self.inputsize = 7
		self.size = self.basissize + self.inputsize
		
		# initialize an arbitrary Koopman Operator with normal distribution
		self.K = np.random.normal(loc=0.0, scale=1, size=self.size*self.size).reshape(self.size, self.size)
		
		# initial data 
		self._psi = self.basis_func(_initmeasure)
		self.data = self._psi.reshape(1, self.size) 
		  
		# initialize K_x & K_u
		self.K_x = self.K[0:self.basissize, 0:self.basissize]                   
		self.K_u = self.K[0:self.basissize, self.basissize:]                    
		self.dt = _dt
		
		# initialize Asum & Gsum
		self.Asum = np.zeros((self.size, self.size))
		self.Gsum = np.zeros((self.size, self.size))

	def compute_operator(self):
		"""
		compute koopman operator K with database
		"""
		A = np.zeros((self.size, self.size))
		G = np.zeros((self.size, self.size))
		M = len(self.data)
		for i in range(M-1):
			A = A + np.dot(self.data[i+1].reshape(self.size, 1), self.data[i].reshape(1, self.size))
			G = G + np.dot(self.data[i].reshape(self.size, 1), self.data[i].reshape(1, self.size))

		A = A/M
		G = G/M
		invG = np.linalg.pinv(G)
		self.K = np.dot(A, invG)
		self.K_x = self.K[0:self.basissize, 0:self.basissize] 
		self.K_u = self.K[0:self.basissize, self.basissize:] 
		
		# update dfdx & dfdu for generalization
		self.dfdx = self.K_x
		self.dfdu = self.K_u
		
		
	def step(self, current_measure):
		"""
		this function performs a single step integration with rk4
		input is current_measure and output is koopman estimate.
		"""
		current_basis = self.basis_func(current_measure)
		state = current_basis[:self.basissize]
		effort = self.basis[(self.size-self.inputsize):]
		k1 = self.f(state, effort)*self.dt
		k2 = self.f(state + k1/2, effort)*self.dt
		k3 = self.f(state + k2/2, effort)*self.dt
		k4 = self.f(state + k3, effort)*self.dt
		"""
		k1 = (np.dot(self.K_x, state) + np.dot(self.K_u, effort))*self.dt
		k2 = (np.dot(self.K_x, state + k1/2) + np.dot(self.K_u, effort))*self.dt
		k3 = (np.dot(self.K_x, state + k2/2) + np.dot(self.K_u, effort))*self.dt
		k4 = (np.dot(self.K_x, state + k3) + np.dot(self.K_u, effort))*self.dt
		"""
		return (k1 + 2. * k2 + 2. * k3 + k4) / 6. + state   

	def update_database(self, newmearsure):
		"""
		add current state to base function
		"""
		self._psi = self.basis_func(newmearsure)
		self.data = np.append(self.data, self._psi.reshape(1, self.size), axis=0)
	
	def f(self, currentstate, currentinput):
		"""
		This is the dynamic function of Koopman model
		"""
		return np.dot(self.K_x, currentstate) + np.dot(self.K_u, currentinput)
	
	def basis_func(self, measure):
		"""
		convert measurement into basis function
		The final length of basis function is basissize. Don't forgot change size of basissize.
		"""
		
		# initial basis function
		self.basis = np.zeros(self.size)
		
		# classify measure into position, velocity and torque
		self.pos = measure[:7]
		self.vel = measure[7:14]
		self.Torque = measure[14:21]
		self.endpos = measure[21:]
		
		# basis function 0-13 are pos & vel
		self.basis[:14] = measure[:14]
		
		# 14-21 are fianl position and orientation of end effector
		self.basis[14:21] = self.endpos
		
		# Last 7 are input/eff
		self.basis[(self.size-self.inputsize):] = self.Torque
		
		# basis function 21-29
		self.basis[21] = 1															# constant variable
		self.basis[22] = self.pos[0]*self.pos[1]
		self.basis[23] = self.pos[1]*self.pos[2]
		self.basis[24] = self.pos[2]*self.pos[3]
		self.basis[25] = self.pos[3]*self.pos[4]
		self.basis[26] = self.pos[4]*self.pos[5]
		self.basis[27] = self.pos[5]*self.pos[6]
		self.basis[28] = (self.pos[0]**2)*(self.pos[1]**2)
		self.basis[29] = (self.pos[1]**2)*(self.pos[2]**2)
		
		# basis function 30-39
		self.basis[30] = (self.pos[2]**2)*(self.pos[3]**2)
		self.basis[31] = (self.pos[3]**2)*(self.pos[4]**2)
		self.basis[32] = (self.pos[4]**2)*(self.pos[5]**2)
		self.basis[33] = (self.pos[5]**2)*(self.pos[6]**2)
		self.basis[34] = (self.pos[0]**3)*(self.pos[1]**3)
		self.basis[35] = (self.pos[1]**3)*(self.pos[2]**3)
		self.basis[36] = (self.pos[2]**3)*(self.pos[3]**3)
		self.basis[37] = (self.pos[3]**3)*(self.pos[4]**3)
		self.basis[38] = (self.pos[4]**3)*(self.pos[5]**3)
		self.basis[39] = (self.pos[5]**3)*(self.pos[6]**3)
		
		# basis function 40-49
		self.basis[40] = self.vel[0]*self.vel[1]
		self.basis[41] = self.vel[1]*self.vel[2]
		self.basis[42] = self.vel[2]*self.vel[3]
		self.basis[43] = self.vel[3]*self.vel[4]
		self.basis[44] = self.vel[4]*self.vel[5]
		self.basis[45] = self.vel[5]*self.vel[6]
		self.basis[46] = (self.vel[0]**2)*(self.vel[1]**2)
		self.basis[47] = (self.vel[1]**2)*(self.vel[2]**2)
		self.basis[48] = (self.vel[2]**2)*(self.vel[3]**2)
		self.basis[49] = (self.vel[3]**2)*(self.vel[4]**2)
		
		# basis function 50-57
		self.basis[50] = (self.vel[4]**2)*(self.vel[5]**2)
		self.basis[51] = (self.vel[5]**2)*(self.vel[6]**2)
		self.basis[52] = (self.vel[0]**3)*(self.vel[1]**3)
		self.basis[53] = (self.vel[1]**3)*(self.vel[2]**3)
		self.basis[54] = (self.vel[2]**3)*(self.vel[3]**3)
		self.basis[55] = (self.vel[3]**3)*(self.vel[4]**3)
		self.basis[56] = (self.vel[4]**3)*(self.vel[5]**3)
		self.basis[57] = (self.vel[5]**3)*(self.vel[6]**3)		

		return self.basis
		
	
def main():
	"""
	Take data from a datasheet and calculate a Koopman Operator
	"""
	
	# time step
	dt = 1./500.
	
	# convert data from datasheets into arrays
	df = pd.read_csv("/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/jointposition_T.csv")
	measuredata = pd.DataFrame.to_numpy(df)
	
	# Get total number of data
	datalength = (len(measuredata[:,0]))
	
	# initial Koopman Operator class
	Koop = KoopmanOperator(measuredata[0,:], dt)
	
	for i in range(1, datalength):
		# update data into Koopman Operator class
		Koop.update_database(measuredata[i,:])
	
	# calculate Koopman Operator
	Koop.compute_operator()
	
	# init Koopman estimate
	K_est = Koop.basis_func(measuredata[0,:])
	# CurrK_est = K_est[:Koop.basissize]
	K_est = K_est[:Koop.basissize]			# take out input from measurement
	
	for i in range(0,datalength-1):
		
		# calculate Koopman estimate 
		NextK_est = Koop.step(measuredata[i,:])
		K_est = np.vstack((K_est, NextK_est))
		# CurrK_est = NextK_est
		
		"""
		NextK_est = Koop.step(measuredata[i,:])
		K_est = np.vstack((K_est, NextK_est))
		"""
		
	# Ouput array into a datasheets
	df = pd.DataFrame(Koop.K)
	path = '/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/KO.csv'
	df.to_csv(path, index=False)
	
	# plot real data and Koopman guess data
	t = np.arange(datalength) * dt
	
	# pos 0
	fig0 = plt.figure(1)
	fig0.suptitle('Joint 1 angle', fontsize=16)
	plt.plot(t, measuredata[:, 0], label='measurement')
	plt.plot(t, K_est[:, 0], label = 'estimate')
	plt.legend()
	
	# pos 1
	fig1 = plt.figure(2)
	fig1.suptitle('Joint 2 angle', fontsize=16)
	plt.plot(t, measuredata[:, 1], label='measurement')
	plt.plot(t, K_est[:, 1], label = 'estimate')
	plt.legend()
	
	# pos 2
	fig2 = plt.figure(3)
	fig2.suptitle('Joint 3 angle', fontsize=16)
	plt.plot(t, measuredata[:, 2], label='measurement')
	plt.plot(t, K_est[:, 2], label = 'estimate')
	plt.legend()
	
	# pos 3
	fig3 = plt.figure(4)
	fig3.suptitle('Joint 4 angle', fontsize=16)
	plt.plot(t, measuredata[:, 3], label='measurement')
	plt.plot(t, K_est[:, 3], label = 'estimate')
	plt.legend()
	
	# pos 4
	fig4 = plt.figure(5)
	fig4.suptitle('Joint 5 angle', fontsize=16)
	plt.plot(t, measuredata[:, 4], label='measurement')
	plt.plot(t, K_est[:, 4], label = 'estimate')
	plt.legend()
	
	# pos 5
	fig5 = plt.figure(6)
	fig5.suptitle('Joint 6 angle', fontsize=16)
	plt.plot(t, measuredata[:, 5], label='measurement')
	plt.plot(t, K_est[:, 5], label = 'estimate')
	plt.legend()
	
	# pos 6
	fig6 = plt.figure(7)
	fig6.suptitle('Joint 7 angle', fontsize=16)
	plt.plot(t, measuredata[:, 6], label='measurement')
	plt.plot(t, K_est[:, 6], label = 'estimate')
	plt.legend()
	
	plt.show()
	
	
		
if __name__=='__main__':
    main()




