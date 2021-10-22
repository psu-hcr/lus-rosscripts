"""
This is a code to calculate possibe Koopman Operator from a known database
"""

from scipy.linalg import solve_continuous_are
import numpy as np
from numpy.linalg import inv, eig
import pandas as pd

class LQR_tinf:
	def __init__(self, _A, _B, _Q, _R):
		self.A = _A
		self.B = _B
		self.Q = _Q
		self.R = _R
		
		# solve Algerbraic Riccati equation
		self.P = solve_continuous_are(self.A,self.B,self.Q,self.R)
		
		# calcuate gain
		self.gain = np.dot(np.dot(-inv(self.R),self.B.T), self.P)
	
	def output(self, _state, _dstate):
		"""
		input current state and desire state
		"""
		output = np.dot(self.gain, (_state - _dstate))
		
		return output
	
def main():
	# take out Koopman operator from excel
	df = pd.read_csv('/home/zxl5344/catkin_ws/src/iiwa_ros/iiwa_ros/scripts/KO.csv')
	K = pd.DataFrame.to_numpy(df)
	length = len(K)
	
	# define input size
	inputsize = 7
	basissize = length - 7
	
	# find A matrix and B matrix
	A = K[:basissize, :basissize]
	B = K[:basissize, basissize:]
	
	# define Q, R matrix
	Q = np.diag(np.concatenate((200*np.ones(3), 0.001*np.ones(basissize-3))))
	#Q = np.diag(200*np.ones(basissize))
	R = np.diag(0.001*np.ones(inputsize))
	
	"""
	testing code
	A = np.array([[1, 0], [0, 1]])
	B = np.array([[1],[1]])
	Q = np.diag([1, 1])
	R = np.array([[1]])
	"""
	w, v = eig(A)
	print("eig of A")
	print(np.amax(w))
	
	# solve LQR in infinte time
	LQR = LQR_tinf(A,B,Q,R)
	
	# print out result
	print(LQR.gain[0])
	
	
	

if __name__=='__main__':
    main()

	
