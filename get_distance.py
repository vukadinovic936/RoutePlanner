import numpy as np
from time import time
from mip import Model, xsum, minimize, BINARY
from itertools import product
from utils import TravellingSalesmanProblem
    
def TSP_ILP(G):
    
	start = time()
	V1 =  range(len(G))
	n, V = len(G), set(V1)
	model = Model()   # binary variables indicating if arc (i,j) is used 
	# on the route or not
	x = [[model.add_var(var_type=BINARY) for j in V] for i in V]   # continuous variable to prevent subtours: each city will have a
	# different sequential id in the planned route except the 1st one
	y = [model.add_var() for i in V]   # objective function: minimize the distance
	model.objective = minimize(xsum(G[i][j]*x[i][j] \
                               for i in V for j in V))
    
   # constraint : leave each city only once
	for i in V:
		model += xsum(x[i][j] for j in V - {i}) == 1   # constraint : enter each city only once
	for i in V:
		model += xsum(x[j][i] for j in V - {i}) == 1   # subtour elimination
	for (i, j) in product(V - {0}, V - {0}):
		if i != j:
			model += y[i] - (n+1)*x[i][j] >= y[j]-n   # optimizing
	model.optimize()   # checking if a solution was found
	if model.num_solutions:
		print('Total distance {}'.format(model.objective_value))
		nc = 0 # cycle starts from vertex 0
		cycle = [nc]
		while True:
			nc = [i for i in V if x[nc][i].x >= 0.99][0]
			cycle.append(nc)
			if nc == 0:
				break
     
	return (model.objective_value, cycle)

distance_matrix = np.load("distance_matrix.npy")
min_path_cost, shortest_path = TSP_ILP(distance_matrix)
print(min_path_cost)
print(shortest_path)

tour = TravellingSalesmanProblem(distance_matrix, 0)
tour.solve()
print(tour.min_path_cost + tour.distance_matrix[0][3])
print(tour.shortest_path)