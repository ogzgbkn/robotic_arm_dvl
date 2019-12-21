import numpy as np

x = 300
y = 0
z = 401

matrix_A = np.array([[-300,-400,300,0,0,0,0,0,0],[0,0,0,-300,-400,300,0,0,0],[0,0,0,0,0,0,300,300,400]])

matrix_B = np.array([[x],[y],[z-1]])

matrix_X = np.dot((np.linalg.inv(matrix_A)),matrix_B)

print(matrix_X)

