import numpy as np
import pandas as pd
from numpy import cos, sin
from KoopmanOperator import KoopmanOperator
from scipy.signal import cont2discrete

class FKsolver:
	"""
	This class use Koopman Operator to generate a diff equation of y_dot = Ay + B(x, phi(x),u)
	It convert this cont. system to digital system y(k+1) = psi*y(k)+gamma*(x, phi(x)).T
	"""
	def __init__(self, _Koop, _dt, _initmeasure):
		self.Koop = _Koop			# koopman Operator class
		self.dt = _dt				# time step
		
		self.A = self.Koop.K[14:21, 14:21]	# A should be 7*7 matrix of y
		self.B = np.append(self.Koop.K[14:21, :14], self.Koop.K[14:21, 21:], axis=1)	# B should be 7*(self.basissize-7) of (x, phi(x), u)
		self.C = np.identity(7)
		self.D = 0
		
		# use sci convert system to digital with zero hold assumption
		self.d_system = cont2discrete((self.A, self.B, self.C, self.D), self.dt, method='zoh')
		self.psi = self.d_system[0]
		self.gamma = self.d_system[1]
		
		# initialize y
		self.y = _initmeasure[14:21]
		
	def step(self, current_measure):
		# convert measurement to koopman state
		Kstate = self.Koop.basis_func(current_measure)
		y_new = np.dot(self.psi, self.y) + np.dot(self.gamma, np.append(Kstate[:14], Kstate[21:]))
		
		# update current y
		self.y = y_new
		
		return self.y
		
def main():
	"""
	This is an example code to verify FK slover
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
	
	# initial FKsolver
	FK = FKsolver(Koop, dt, measuredata[0,:])
	
	# 1 step
	FK.step(measuredata[1,:])
	

if __name__=='__main__':
    main()
