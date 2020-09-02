import numpy as np

def createTable(values, polyDegree):
	X = np.zeros((len(values), (polyDegree + 1)))

	for i in range(0, len(values)):
		for d in range(0, (polyDegree + 1)):
			X[i][d] = (values[i]**d)
	
	W = (X.T).dot(X)
	Winv = np.linalg.inv(W)
	return (Winv.dot(X.T))
