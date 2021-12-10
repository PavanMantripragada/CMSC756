import sympy as sp
import numpy as np

j1,j2,b1,b2 = sp.symbols('j1','j2','b1','b2')

A = np.array([[0,1/j1,-b1/j1,b1**2/j1],
              [0,1/j2,-b2/j2,b2**2/j2],
              [1/j1,-b1/j1,b1**2/j1,-b1**3/j1],
              [1/j2,-b2/j2,b2**2/j2,-b2**3/j2]])

print(np.linalg.det(A))