"""
This is a class for differential drive vehicle from HW1
"""

import numpy as np
from math import pi, cos, sin, pow
import matplotlib.pyplot as plt

class DDV:
	def __init__(self):
		pass
		
	def f(self, x, u):
		"""
		input current theta and current u
		return f(theta, u) in n*1 2D array
		"""
		return np.array([cos(x[2])*u[0] , sin(x[2])*u[0], u[1]]) 
	
	def dfdx(self, x, u):
		"""
		input current theta and current u
		return dfdtheta(theta, u) in n*1 2D array
		"""
		self.A = np.array([[0, 0, -sin(x[2])*u[0]], [0, 0, cos(x[2])*u[0]], [0, 0, 0]])
		return self.A
	
	def dfdu(self, x, u):
		"""
		input current theta and current u
		return dfdu(theta, u) in n*1 2D array
		"""
		self.B = np.array([[cos(x[2]), 0], [sin(x[2]), 0], [0, 1]])
		return self.B
	
	def step(self, xinit, u, dt):
		"""
		step function with initial condition xinit and current input u
		"""
		k1 = self.f(xinit, u)*dt
		k2 = self.f(xinit + k1/2, u)*dt
		k3 = self.f(xinit + k2/2, u)*dt
		k4 = self.f(xinit + k3, u)*dt
		new_x = (k1 + 2. * k2 + 2. * k3 + k4) / 6. + xinit
		return new_x

def desire_traj(t):
	"""
	define desire traj equation
	"""
	x_d = 4/(2*pi) * t
	y_d = 0
	theta_d = pi/2
	
	return np.array([x_d, y_d, theta_d])
	
	
	
