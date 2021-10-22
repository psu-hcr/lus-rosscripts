"""
This is a class for LQR cost class
"""

from scipy.linalg import solve_continuous_are
import numpy as np
from numpy.linalg import inv, eig
import pandas as pd

class LQR_l:
	def __init__(self, _Q, _R, _dstate, _dt):
		"""
		Q: state gain
		R: input gain
		_dstate: desire traj func wrt time
		dt: time step
		"""
		self.Q = _Q
		self.R = _R
		self.dstate = _dstate
		self.dt = _dt
	
	def cost(self, t_period, traj, u):
		"""
		This function take time array, actual traj & input. It returns the cost.
		t_period: time array
		traj: actual traj
		u: actual input
		dt: time step
		return: cost 
		"""
		# initialize cost
		J = 0.
		
		# integrate l with RK4
		for i in range(len(t_period)-1):
			k1err = traj[:, i] - self.dstate(t_period[i])
			k2err = traj[:, i] - self.dstate(t_period[i] + self.dt/2)
			k3err = traj[:, i] - self.dstate(t_period[i] + self.dt/2)
			k4err = traj[:, i+1] - self.dstate(t_period[i] + self.dt)
			k1 = np.dot(k1err.T, np.dot(self.Q, k1err)) + np.dot(u[:, i].T, np.dot(self.R, u[:, i]))
			k2 = np.dot(k2err.T, np.dot(self.Q, k2err)) + np.dot(u[:, i].T, np.dot(self.R, u[:, i]))
			k3 = np.dot(k3err.T, np.dot(self.Q, k3err)) + np.dot(u[:, i].T, np.dot(self.R, u[:, i]))
			k4 = np.dot(k4err.T, np.dot(self.Q, k4err)) + np.dot(u[:, i].T, np.dot(self.R, u[:, i+1]))
			J = J + (1 / 6.) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)*self.dt
			"""
			print(k1, k2, k3, k4)
			print(str(i) + "th J")
			print(J)
			"""
		return J
	def dldx(self, t, traj_c):
		"""
		This function is derivative of l wrt state x
		t: current time
		traj_c: actual current traj
		return: dldx at time t 
		"""
		error = traj_c.flatten() - self.dstate(t).flatten()
		return np.dot(self.Q, error)
	
	
def main():
	pass
	
if __name__=='__main__':
    main()

	
