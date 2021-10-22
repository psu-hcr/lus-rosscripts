# This code is used to calculate SAC controller

import numpy as np
from numpy.linalg import multi_dot
from math import pi, cos, sin, pow
from matplotlib import pyplot as plt
import pandas as pd
from math import ceil
from LQcostfunc import LQR_l
from Drive_vehicle import DDV, desire_traj
    
class SAC:
	"""
	Sequnetial action controller (SAC) class
	"""
	def __init__(self, _J_min, _delat_tinit, _u1, _u_sat, _omega, _T, _ts, _tcalc, _kmax, _gamma, _beta, _sys, _l, _R):
		"""
		input meaning
		J_min: 	minimum change in cost
		delat_tinit:	default control duration
		u1:		nominal control
		u_sat:		saturation input
		omega:		scale factor
		T:		prediction horizon 
		ts:		sampling time
		tcalc		the max time for iterative control calculation
		k_max		the max backtracking iterations
		gamma		the feedback law to calculate alpha_d
		beta		the coefficient of time to calculate the J_tao
		sys		the dynamic system we plan to control. It should be a state space function
		l		the cost function 
		R		the control cost gain matrix on J_tao
		"""
		self.J_min = _J_min
		self.delat_tinit = _delat_tinit
		self.u_sat = _u_sat
		self.omega = _omega
		self.T = _T
		self.ts = _ts
		self.tcalc = _tcalc
		self.kmax = _kmax
		self.gamma = _gamma
		self.beta = _beta 
		self.sys = _sys
		self.l = _l
		self.R = _R
	
		# initialize current time
		self.tcurr = 0.
		self.t0 = self.tcurr
		self.tf = self.tcurr + self.T
		self.t = np.arange(self.t0, self.tf + self.ts, self.ts)
		
		# initial uniform input 
		self.u1 = np.dot(_u1.reshape((len(_u1),1)), np.ones((1, len(self.t))))
		self.u1norm = _u1
		self.uoutput = self.u1[1, :]
		
	
	def calcinput(self, t_curr, currentstate, _sys):
		"""
		this function take current time and current koopman state.
		It update the current dynamic model based on current Koopman system
		return current input for dynamic system
		"""
		
		# update time
		self.tcurr =  t_curr
		self.t0 = self.tcurr
		self.tf = self.tcurr + self.T
		self.t = np.arange(self.t0, self.tf + self.ts, self.ts)
		
		# clip & fix the length of t
		self.t = self.t[:int(self.T/self.ts)]
		
		# update dynmaic system model and dfdx & h
		self.sys = _sys
		self.h = self.sys.dfdu(currentstate, self.uoutput)					
		self.dfdx = self.sys.dfdx(currentstate, self.uoutput)				
		
		# simulation traj
		self.sim(currentstate, self.u1)
		
		# solve rho
		self.rho = np.zeros((len(currentstate), len(self.t)))
		self.rho[:, len(self.t)-1] = np.zeros(len(currentstate))
		for i in range(len(self.t)-1, 0, -1):
			k1 = self.rhodot(self.rho[:, i], self.xtraj[:, i], self.u1[:, i], self.t[i]) * (-self.ts)
			k2 = self.rhodot(self.rho[:, i] + k1/2, self.xtraj[:, i], self.u1[:, i], self.t[i]) * (-self.ts)
			k3 = self.rhodot(self.rho[:, i] + k2/2, self.xtraj[:, i], self.u1[:, i], self.t[i]) * (-self.ts) 		
			k4 = self.rhodot(self.rho[:, i] + k3, self.xtraj[:, i], self.u1[:, i], self.t[i]) * (-self.ts) 
			self.rho[:, i-1] = (k1 + 2. * k2 + 2. * k3 + k4)/ 6. + self.rho[:, i]
			#print(str(i-1) + "th rho")
			#print(self.rho[:, i - 1])
		
		# compute J_1_init
		self.J1_init = self.l.cost(self.t, self.xtraj, self.u1)		
		
		#print('J')
		#print(self.J1_init)
		
		# calculate sensitivity alpha
		self.alpha_d = self.gamma * self.J1_init
		#print("alpha_d")
		#print(self.alpha_d)
		
		# find tao when u2star(tao) is min		
		current_i = ceil(self.tcalc/self.ts)		# calculate the first time step when it is larger than t_calc
		self.minJtao = self.J_tao(current_i)
		self.tao = self.t[current_i]
		for i in range(current_i, len(self.t)):
			#print(self.J_tao(i))						
			if self.J_tao(i) < self.minJtao:
				self.minJtao = self.J_tao(i)
				self.tao = self.t[i]
		#print('tao')
		#print(self.tao)
		#print('minJtao')
		#print(self.minJtao)
    		
		# initial k and J1_new
		self.k = 0
		self.J1_new = 10e8
		
		# store old u1
		self.u1_old = self.u1
		
		# search for lambda
		while self.J1_new - self.J1_init > self.J_min and self.k <= self.kmax:

			# restore the data
			self.u1 = self.u1_old

			# calculate lambda1, tao_0 & tao_f
			self.lambda1 = (self.omega**self.k)*self.delat_tinit
			self.tao_0 = self.tao - self.lambda1/2
			if self.tao_0 < self.t0:
				self.tao_0 = self.t0
			self.tao_f = self.tao + self.lambda1/2
			if self.tao_f > self.tf:
				self.tao_f = self.tf
				
			#print('lambda1, tao_0, tao_f')
			#print(self.lambda1, self.tao_0, self.tao_f)
			
			# add u2star to u1 and saturate
			for i in range(int((self.tao_0-self.tcurr)/self.ts), int((self.tao_f-self.tcurr)/self.ts)):

				# saturate u2star
				u_new = self.u2star(i)
				self.u1[0, i] = np.clip(u_new[0], -self.u_sat[0], self.u_sat[0])
				self.u1[1, i] = np.clip(u_new[1], -self.u_sat[1], self.u_sat[1])
			
			# re_simulation
			self.sim(currentstate, self.u1)
			
			# calculate new cost
			self.J1_new = self.l.cost(self.t, self.xtraj, self.u1)
			#print("J1_new")
			#print(self.J1_new)
			
			# update iteration counter	
			self.k = self.k+1
			
		self.uoutput = self.u1[:, 0]
		
		# shift u1 to current time step
		self.u1 = np.append(self.u1[:,1:], self.u1norm.reshape((len(self.u1norm),1)), axis=1)
		
		return self.uoutput
		
	def sim(self, currentstate, u):
		"""
		this function simulate the system based on current measurement
		currentmeasure is the Koopman State
		u is the input in horizon
		"""
		
		# initlize the simulation result
		self.xtraj = np.zeros((len(currentstate), len(self.t)))
		
		# inital condition is current measurement
		xcurr = currentstate
		
		for i in range(len(self.t)):
			k1 = self.sys.f(xcurr, u[:, i])*self.ts
			k2 = self.sys.f(xcurr + k1/2, u[:, i])*self.ts
			k3 = self.sys.f(xcurr + k2/2, u[:, i])*self.ts
			k4 = self.sys.f(xcurr + k3, u[:, i])*self.ts
			self.xtraj[:, i] = (k1 + 2. * k2 + 2. * k3 + k4) / 6. + xcurr
			xcurr = self.xtraj[:, i]
		# print("xtraj")
		# print(self.xtraj)
		return self.xtraj
	    	
	def rhodot(self, current_rho, current_traj, current_u, current_time):
		"""
		This function define rhodot. It is used for numerically
		"""
		# print("rhodot")
		# print(self.sys.dfdx(current_traj, current_u).T, current_rho, self.l.dldx(current_time, current_traj))
		return - np.dot(self.dfdx.T, current_rho) - self.l.dldx(current_time, current_traj)
	
	def Lambda(self, i):
		"""
		This is the lambda function for calculate u2_start at ith time in time period
		lambda = h.T*rho*rho.T*h
		"""
		Rho = self.rho[:, i].reshape((len(self.rho[:, i]), 1))
		RhoT = self.rho[:, i].reshape((1, len(self.rho[:, i])))
		return self.h.T.dot(Rho).dot(RhoT).dot(self.h)
	
	def u2star(self, i):
		"""
		calculate u2* at ith time in time period
		"""
		# h = self.sys.dfdu(self.xtraj[:, i], self.u1[:, i])
		"""
		print("u2star")
		print(self.Lambda(i))
		print(self.R.T) 
		print(self.u1[:, i])
		print(h.T)
		print(self.rho[:, i])
		"""
		return np.dot(np.linalg.inv(self.Lambda(i) + self.R.T), (np.dot(self.Lambda(i), self.u1[:, i]) + np.dot(self.h.T, self.rho[:, i]) * self.alpha_d))
		
	def dJ1_dlambda(self, i):
		"""
		Define dJ1_dlambda at ith time in time period
		"""
		# h = self.sys.dfdu(self.xtraj[:, i], self.u1[:, i])
		return np.dot(self.rho[:, i].T, (np.dot(self.h, self.u2star(i)) - np.dot(self.h, self.u1[:, i])))		# problem might be dJ1_dlambda
		
	def J_tao(self, i):
		"""
		second cost function J_tao at ith time in time period
		"""
		# print("J_tao")
		# print(np.linalg.norm(self.u2star(i)), self.dJ1_dlambda(i), pow(i*self.ts, self.beta))
		return np.linalg.norm(self.u2star(i)) + self.dJ1_dlambda(i) + pow(i*self.ts, self.beta)
		
def main():
	"""
	This is a test model for SAC controllor. It use HW1 Prob.5 model for testing
	"""
	
	J_min = 0.0001  	# minimum change in cost
	t_curr = 0  		# current time
	t_init = 0.1  		# default control duration
	omega = 0.5  		# scale factor
	T = 2  		# predictive horizon
	ts = 0.01  		# sampling time
	t_calc = 0.  		# the max time for iterative control calculation
	k_max = 5  		# the max backtracking iterations
	gamma = -9.  		# the coefficient for first order sensitivity alpha
	beta = 0.7		# the coefficient of time to calculate the J_tao
	u1 = np.array([1, -0.5])				# nomial input
	u_sat = np.array([5,5]) 				# input saturation
	Q = np.array([[2, 0, 0], [0, 2, 0], [0, 0, 0.01]])  	# Q in cost function
	R = np.array([[0.01, 0], [0, 0.01]])  		# R in cost function
	
	# the end time t_end
	t_end = 2*pi
	
	# define x_init
	x_init = np.array([0, 0, pi / 2])
	
	# define system
	sys = DDV()	
	
	# define cost function					
	l = LQR_l(Q, R, desire_traj, ts)
	
	# define SAC controller
	sac = SAC(J_min, t_init, u1, u_sat, omega, T, ts, t_calc, k_max, gamma, beta, sys, l, R)
	
	# initialize counter and current x
	i = 0
	currentx = x_init
	x = x_init.reshape((len(x_init),1))
	x1 = x_init.reshape((len(x_init),1))
	U = np.zeros((2,1))
	
	# simulation
	while t_curr<t_end:
		 
		print(t_curr)
		u = sac.calcinput(t_curr, currentx, sys)
		U = np.append(U, u.reshape((len(u), 1)), axis=1)	
		newx = sys.step(currentx, u, ts)
		newx1 = desire_traj(t_curr)
		t_curr = t_curr + ts
		# print(u)
		x = np.append(x, newx.reshape((len(newx), 1)), axis=1)
		x1 = np.append(x1, newx1.reshape((len(newx), 1)), axis=1)
		currentx = newx
	
	# Ouput x array into a datasheets
	df = pd.DataFrame(x)
	path = '/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/Drive_vehile.csv'
	df.to_csv(path, index=False)
	
	# plot traj and input
	fig, (ax1, ax2) = plt.subplots(2)
	ax1.plot(x[0,:], x[1,:], label='measurement')
	ax1.plot(x1[0,:], x1[1,:], label='origin')
	ax1.legend()
	
	ax2.plot(U[0, :], label='u1')
	ax2.plot(U[1, :], label='u2')
	ax2.legend()
	
	plt.show()
	
if __name__=='__main__':
    main()
