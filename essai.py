import numpy as np

liste = [1,2,3,4,5,6,78,9]

A = np.zeros((len(liste),2))

A[5,1] = 1


print(A[5, :])