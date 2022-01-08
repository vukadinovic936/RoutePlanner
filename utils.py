import googlemaps
import numpy as np
import os
import re
from mip import Model, xsum, minimize, BINARY
from time import time
from itertools import product

def MyTSP(G):
    
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

class TravellingSalesmanProblem:
    def __init__(self, distance, start):
        self.distance_matrix = distance
        self.start_city = start
        self.total_cities = len(distance)

        self.end_state = (1 << self.total_cities) - 1
        self.memo = [[None for _col in range(1 << self.total_cities)] for _row in range(self.total_cities)]

        self.shortest_path = []
        self.min_path_cost = float('inf')

    def solve(self):
        self.__initialize_memo()

        for num_element in range(3, self.total_cities + 1):

            for subset in self.__initiate_combination(num_element):

                if self.__is_not_in_subset(self.start_city, subset):
                    continue

                for next_city in range(self.total_cities):

                    if next_city == self.start_city or self.__is_not_in_subset(next_city, subset):
                        continue

                    subset_without_next_city = subset ^ (1 << next_city)
                    min_distance = float('inf')

                    for last_city in range(self.total_cities):

                        if last_city == self.start_city or \
                                last_city == next_city or \
                                self.__is_not_in_subset(last_city, subset):
                            continue

                        new_distance = \
                            self.memo[last_city][subset_without_next_city] + self.distance_matrix[last_city][next_city]

                        if new_distance < min_distance:
                            min_distance = new_distance

                    self.memo[next_city][subset] = min_distance

        self.__calculate_min_cost()
        self.__find_shortest_path()

    def __calculate_min_cost(self):
        for i in range(self.total_cities):

            if i == self.start_city:
                continue

            path_cost = self.memo[i][self.end_state]

            if path_cost < self.min_path_cost:
                self.min_path_cost = path_cost

    def __find_shortest_path(self):
        state = self.end_state

        for i in range(1, self.total_cities):
            best_index = -1
            best_distance = float('inf')

            for j in range(self.total_cities):

                if j == self.start_city or self.__is_not_in_subset(j, state):
                    continue

                new_distance = self.memo[j][state]

                if new_distance <= best_distance:
                    best_index = j
                    best_distance = new_distance

            self.shortest_path.append(best_index)
            state = state ^ (1 << best_index)

        self.shortest_path.append(self.start_city)
        self.shortest_path.reverse()

    def __initialize_memo(self):
        for destination_city in range(self.total_cities):

            if destination_city == self.start_city:
                continue

            self.memo[destination_city][1 << self.start_city | 1 << destination_city] = \
                self.distance_matrix[self.start_city][destination_city]

    def __initiate_combination(self, num_element):
        subset_list = []
        self.__initialize_combination(0, 0, num_element, self.total_cities, subset_list)
        return subset_list

    def __initialize_combination(self, subset, at, num_element, total_cities, subset_list):

        elements_left_to_pick = total_cities - at
        if elements_left_to_pick < num_element:
            return

        if num_element == 0:
            subset_list.append(subset)
        else:
            for i in range(at, total_cities):
                subset |= 1 << i
                self.__initialize_combination(subset, i + 1, num_element - 1, total_cities, subset_list)
                subset &= ~(1 << i)

    @staticmethod
    def __is_not_in_subset(element, subset):
        return ((1 << element) & subset) == 0


def get_distance_matrix(city_names, preload=False):
	""" 
		Take an array of strins as input and returns a 2D array of distances between cities
	"""
	if("distance_matrix.npy" in os.listdir()) and preload:
		# don't spend requests!!
		return np.load("distance_matrix.npy")
	else:
		gmaps = googlemaps.Client(key = 'AIzaSyD4nE1eqouTNUPqSkLogyZMmjWPkuTZtU4')
		save_res = gmaps.distance_matrix( origins = city_names, destinations = city_names )
		distance_matrix = np.zeros((len(city_names),len(city_names)))

		for i in range(len(city_names)):
			for j in range(len(city_names)):
				distance_matrix[i][j] = save_res['rows'][i]['elements'][j]['distance']['value']

		return distance_matrix

def get_distance(city_names, printPath=False):
    """
    Takes an array of strings city names
	returns the best order to visit them and min distance
	"""
    distance_matrix = get_distance_matrix(city_names)

    min_path_cost, shortest_path = MyTSP(distance_matrix)

    if printPath:
        print("Shortest path :", shortest_path)
        print("Minimum path cost :", min_path_cost)
        for i in range(len(city_names)):
            print(city_names[shortest_path[i]].split(',')[0], end=" ")

    paths = [""]*(len(city_names)+1)
    for i in range(len(city_names)+1):
        paths[i] = city_names[shortest_path[i]].split(',')[0]

    links = []
    for i in range(len(city_names)):
        s1 = re.sub("[^0-9a-zA-Z]+", "+", paths[i])
        s2 = re.sub("[^0-9a-zA-Z]+", "+", paths[i+1])
        links.append(f"https://www.google.com/maps/dir/{s1}/{s2}/")

    for i in range(len(city_names)):
        paths[i] = paths[i]+" - " + paths[i+1]

    block = []
    for i in range(len(city_names)):
        entry = {"path":paths[i], "link":links[i]}
        block.append(entry)
    return {'shortest_path': block, 'min_distance': min_path_cost/1000, 'links': links }

class Singleton:
	__instance = None
	@staticmethod 
	def getInstance():
		""" Static access method. """
		if Singleton.__instance == None:
			Singleton()
		return Singleton.__instance
	def __init__(self):
		""" Virtually private constructor. """
		if Singleton.__instance != None:
			raise Exception("This class is a singleton!")
		else:
			Singleton.__instance = self
		Singleton.city_names = []

if __name__ == "__main__":
    print("s")

city_names = ["Nis, Serbia",
             "Kraljevo, Serbia",
             "Jagodina, Serbia",
             "Novi Sad, Serbia",
             "Cacak, Serbia", 
             "Babusnica, Serbia",
             "Dimitrovgrad, Serbia"]