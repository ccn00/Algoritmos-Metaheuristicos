# %% [markdown]
# <br><br><br>
# <h1><font color="#B30033" size=5>Intelligent Systems - Course 2022-2023</font></h1>
# 
# 
# 
# <h1><font color="#B30033" size=5>Lab 1: State Space Search</font></h1>
# 
# 
# <br>
# <div style="text-align: left">
# <font color="#4E70BE" size=3>Lecturers:</font><br>
# <ul>
#   <li><font color="#4E70BE" size=3>Juan Carlos Alfaro Jiménez (JuanCarlos.Alfaro@uclm.es)</font><br></li>
#   <li><font color="#4E70BE" size=3>Guillermo Tomás Fernández Martín (Guillermo.Fernandez@uclm.es)</font><br></li>
#   <li><font color="#4E70BE" size=3>Mª Julia Flores Gallego (Julia.Flores@uclm.es)</font><br></li>
#   <li><font color="#4E70BE" size=3> José Antonio Gámez Martín (Jose.Gamez@uclm.es)</font><br></li>
#   <li><font color="#4E70BE" size=3> Ismael García Varea (Ismael.Garcia@uclm.es)</font><br></li>
#   <li><font color="#4E70BE" size=3> Luis González Naharro (Luis.GNaharro@uclm.es)</font><br></li>
# </ul>
# </div>

# %% [markdown]
# --------------------
# ## 1. Introduction
# 
# In this assignment, we will put into practice the techniques for searching the state of spaces. To do that, some of the algorithms seen in units two and three will be implemented and used to solve a classical problem: searching paths on maps where the locations will be cities identified by their latitude and longitude values, as in most [geographical systems](https://en.wikipedia.org/wiki/Geographic_coordinate_system).
# 
# We will also analyze and compare the performance of the algorithms by running them over different instances of the problem, and providing distinct initial and goal states.

# %% [markdown]
# ## 2. Problem Description
# 
# The concept of map we will use is simple: it can be represented by a graph with cities and undirected connections (that is, they work exactly the same in both ways), which indicate that there is a specific road between them two, which can be used for moving from one to the other in one action. Also, these edges will have associated a number of units, which tipycally represents the real/driving distance between the two cities.
# 
# We opted to use realistic maps so that the cities are real, and the driving distances are also extracted from a navigation API. But the connections are established so that only some of them are taken.
# 
# A map is a particular problem, but then we need to answer queries where there will be an initial state and a final state. In the most simple way, both will be the location/city. So to reach city B from A, we would aim at finding the finding the shortest path (smallest cost).

# %% [markdown]
# ## 3. Assignment Development
# 
# During the development of the assignment, you will be given a set of maps, in which you should perform a list of searches. The dimensionality, both in the number of cities and in their connectivity, will be variable, and your algorithms should be efficient enough to work properly in all of them. Some other scenarios (maps and searches) will be kept for the evaluation/correction/interview, so make your code general enough to load them easily.

# %% [markdown]
# ### 3.1 Input Problems
# 
# Every scenario will have associated a JSON file with the following structure: 
# 
# ```JSON
# {
#     "map": {
#         "cities": [
#             {
#                 "id": id_city_0,
#                 "name": name_city_0,
#                 "lat": latitude_city_0,
#                 "lon": longitude_city_0
#             }
#         ],
#         "roads": [
#             {
#                 "origin": origin_city_id,
#                 "destination": destination_city_id,
#                 "distance": road_distance
#             }
#         ]
#     },
#     "departure": departure_city_id,
#     "goal": goal_city_id
# }
# ```
# 
# There are three general keys in the JSON: 
# 
# - `map`: A dictionary that represents the map of the problem.
# - `departure`: The trip departure city id, this is, the initial state.
# - `goal`: The trip goal city id, this is, the end state.
# 
# In the map dictionary, there are two keys: 
# - `cities`: An array with the cities, this is, the nodes of the map.
# - `roads`: An array with the roads, this is, the connections between nodes.
# 
# Finally, a city is represented as: 
# - `id`: The id of the city, used for most operations.
# - `name`: The name of the city, used for representing the solution in a human readable way.
# - `lat`: The latitude of the city, used for plotting representations.
# - `lon`: The longitude of the city, used for plotting representations.
# 
# And a road is represented as: 
# - `origin`: The origin city id.
# -  `destination`: The destination city id.
# -  `distance`: The distance in kilometers using that road.
# 
# The roads will be directed but the JSON will have symmetric roads, meaning that there will be a road from A to B and a second road from B to A.

# %% [markdown]
# ## 4. Work plan

# %% [markdown]
# ### 4.1 Problem Formalization and Examples
# 
# First of all, path finding in maps must be formalized as a problem of search in the space of states, by defining its basic elements. All implementations must refer to search in graphs, so it is important to take into consideration that repeated states must be controlled. 
# 

# %% [markdown]
# ### 4.2 Implementation
# 
# Below, you will have the class structure regarding the problem at hand. You will have to complete the following classes by implementing the algorithms studied in theory. Add all your imports in this cell to have the notebook properly organized.

# %%
# =============================================================================
# Imports
# =============================================================================

# Standard
import json
import random
import itertools
from abc import ABC, abstractmethod
import math

#Añadidos por mi
from geopy.distance import geodesic 
from queue import PriorityQueue

from time import perf_counter
# Third party
import geopandas as gpd
from shapely.geometry import Point

# %% [markdown]
# #### Class `Action` # Representación de carreteras en JSON
# This class provides the **representation of the actions** that will be performed by the traveler. An action is defined by the `origin` and `destination` of the trip, as well as the cost of the action.
# 
# Methods you must add: 
# 
# - `__init__(self, args)`: Constructor of the class, with the necessary arguments
# 
# Methods recommended: 
# 
# -- repr es mas utilizado cuando para el desarrollo y depuración
# - `__repr__(self)`: String representation of the objects. Useful to debug list of `Action`
# 
# --str es mas utilizado para el usuario final 
# - `__str__(self)`: Method used when `print(Action)` is called. Useful to debug single `Action`

# %%
class Action:
    def __init__(self, origin, destination, distance):
        #Lista de acciones
        self.origin = origin
        self.destination = destination
        self.distance = distance


    # Lista de acciones
    def __repr__(self):
        return 'Action(Ciudad origen: %s, Ciudad destino: %s, Distancia: %s)' % (self.origin, self.destination, self.distance)
        # return f'origin: {self.origin} : {self.destination} Coste del viaje: {self.tripCost}'
    # Devuelve una acción
    def __str__(self):
        return f'Ciudad origen: {self.origin} Ciudad destino: {self.destination} Distancia: {self.distance}'
    

# %% [markdown]
# #### Class `State`
# 
# This class provides the **representation of a state** in the search space. In this problem, a state is defined by the **city** in which the traveler is in a particular moment. Note that the map itself does not need to be part of the state given that it does not change during the search.
# 
# Methods you must add: 
# 
# - `__init__(self, args)`: Constructor of the class, with the necessary arguments
# - `__eq__(self, other)`: Equals method. Used for hash table comparison
# - `__hash__(self)`: Hashing method. Used to generate unique hashes of the objects. Used for hash table.
# - `apply_action(self, args)`: given a valid `Action`, returns the new `State` generated from applying the `Action` to the current `State`. 
# 
# Methods recommended: 
# 
# - `__repr__(self)`: String representation of the objects. Useful to debug list of `State`
# - `__str__(self)`: Method used when `print(State)` is called. Useful to debug single `State`

# %% [markdown]
# 

# %%
class State:

    # 0: [origin 0, destination 1, trip_cost 96.9,
    # origin 0, destination 5, trip_cost 72.1]

    # 0 Es la ciudad actual
    # [origin 0, destination 1, trip_cost 96.9, origin 0, destination 5, trip_cost 72.1] actionsES QUE SE PUEDEN REALIZAR DESDE ESTA CIUDAD


    def __init__(self, city, actions = None):
        # Ciudad en la que se encuentra
        self.city = city
        # Acciones que podemos realizar desde esta ciudad
        self.actions = actions

    #Sobreescribimos la funcion eq para que compare solo la ciudad    
    def __eq__(self, other):
        # Comprobamos si es una instancia por si hace otra pregunta para que no de un error si es comparado con otro tipo de dato
        if isinstance(other, State):
            return self.city == other.city  
        return False

    # Se debe crear la funcion hash puesto que nuestra clase sobreescribe la funcion eq. Al utilizar __eq__ en esta clase,
    # se convierte unhashable, es decir, no seras capaz de usar objetos de tipo mapping, no se podran utilizar como claves de diccionario
    # o como elementos de un conjunto.
    def __hash__(self):
        return hash(self.city)

    # Dada una acción valida, devuelve un nuevo estado generado de aplicar la acciones del actual estado
    def apply_action(self, actions):
    # Comprobar que la acciones es valida
        if actions not in self.actions:
            raise ValueError('La accion no es valida')
        # Devuelve un nuevo estado
        return State(actions.destination , actions)    
        
    # Devuelve lista de estados
    def __repr__(self):
        return f'Esta en la ciudad {self.city}'
        
    # Devuelve estado
    def __str__(self):
        return f'Esta en la ciudad {self.city}'

# %% [markdown]
# #### Class `Node`. 
# This class provides a **representation of a node** in the search graph. A `Node` is defined by the `State` it represents, its parent `Node` and the `Action` taken to reach the current `Node` from the parent `Node`. 
# 
# **It can also have any other attributes necessary for the search algorithms**.
# 
# Methods you must add: 
# 
# - `__init__(self, args)`: Constructor of the class, with the necessary arguments
# - `__eq__(self, other)`: Equals method. Used for hash table comparison
# 
# Methods recommended: 
# 
# - `__repr__(self)`: String representation of the objects. Useful to debug list of `Node`
# - `__str__(self)`: Method used when `print(Node)` is called. Useful to debug single `Node`

# %%
class Node:

    def __init__(self, state, parent, action, path_cost, depth):

        # Estado del espacio de estados que corresponde con el nodo
        self.state = state

        # Nodo en el árbol de búsqueda que ha generado este nodo
        self.parent = parent

        # Accion que se aplicará al padre para generar al nodo
        self.action = action

        # Coste denotado por g(n), de un camino desde el estado inicial al nodo, indicado por los punteros a los padres y la profundidad
        self.path_cost = path_cost

        # Profundidad del nodo en el árbol de búsqueda
        self.depth = depth

        
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.state == other.state
        return False

    def __hash__(self):
        return hash(self.state)
        
    def __repr__(self):
        return f'estado: {self.state}, Nodo padre: {self.parent}, Accion tomada: {self.action}, Coste: {self.path_cost}, Profundidad: {self.depth}'
    def __str__(self):
        pass
    

# %% [markdown]
# #### Class `Problem`
# This class provides the **representation of the search problem**. This class reads from a file an instance of the problem to be solved. It is defined by the `map`, the `initial_state` and the `final_city` as well as several auxiliary structures to plot the problem. This class must also generate all the `Actions` that can be taken in the problem. It is recommended to store them in a dictionary of the form `{'origin_city_id': [action_1, action_2, action_3, ...]}`. 
# 
# Methods you must add: 
# 
# - `__init__(self, args)`: Constructor of the class, with the necessary arguments
# - Method to generate the `Action` dictionary from the map
# - Method to check if a `State` is the goal state
# 
# Methods recommended: 
# 
# - Method to get the `Actions` associated to a particular `Node`
# 
# The class `Problem` also has an auxiliary method to plot the whole map and to plot the map with a solution of the problem formed by a list of actions. This can be called as `problem.plot_map()` for the whole map and all its connections, or as `problem.plot_map([action_1, action_2, ...])` to plot just the solution.

# %%
class Problem:

    def __init__(self, problem):
        # Method to read the problem JSON file
        # with open(filename, 'r', encoding='utf8') as file:
        #     problem = json.load(file)
        
        # Auxiliary structures for the plot_map function
        self.cities = {city['id']: city for city in problem['map']['cities']}
        self.gdf = gpd.GeoDataFrame(problem['map']['cities'])
        self.gdf['Coordinates'] = list(zip(self.gdf.lon, self.gdf.lat))
        self.gdf['Coordinates'] = self.gdf['Coordinates'].apply(Point)
        self.gdf.set_geometry('Coordinates', inplace=True)

        # Estado inicial
        self.departure = problem['departure']
        # Estado objetivo
        self.goal = problem['goal']
        # Lista de acciones
        self.roads = problem['map']['roads']
        # Diccionario de acciones asociados a cada ciudad
        self.diccionario_acciones = {}


    def __str__(self):
        return f'Ciudad origen: {self.cities[self.departure]} ciudad destino: {self.cities[self.goal]}'

    # Diccionario de acciones que se pueden realizar
    def create_action_dictionary(self):         
        if len(self.diccionario_acciones) == 0:                      #Comprobamos si diccionario esta vacio
            for x in self.roads:
                if x['origin'] not in self.diccionario_acciones:     #Si la clave todavia no esta la introducimos
                    self.diccionario_acciones[x['origin']] = [Action(x.get('origin'), x.get('destination'), x.get('distance'))]
                elif x['origin'] in self.diccionario_acciones:       #Si la clave ya esta en el dic añadimos el valor
                    self.diccionario_acciones[x['origin']].append(Action(x.get('origin'), x.get('destination'), x.get('distance')))

    
    # Comprueba si la ciudad actual es la ciudad objetivo devuelve V o F
    def comprueba_objetivo(self, estado):
        return estado == self.goal

    def get_actions(self, state):
        # Devuelve las acciones que se pueden realizar desde el estado actual
        if len(self.diccionario_acciones) == 0: 
            self.create_action_dictionary()
        return self.diccionario_acciones[state]


    def plot_map(self, action_list=None, world_name='Spain'):
        world = gpd.read_file(gpd.datasets.get_path('naturalearth_lowres'))
        city_ids = {self.cities[city]['name']: city for city in self.cities}
        # We restrict to Spain.
        ax = world[world.name == world_name].plot(
            color='white', edgecolor='black',linewidth=3,figsize=(100,70))

        self.gdf.plot(ax=ax, color='red',markersize=500)
        for x, y, label in zip(self.gdf.Coordinates.x, self.gdf.Coordinates.y, self.gdf.name):
            ax.annotate(f'{city_ids[label]} -- {label}', xy=(x, y), xytext=(8, 3), textcoords="offset points",fontsize=60)
        roads = itertools.chain.from_iterable(self.diccionario_acciones.values())

        for road in roads:
            slat = self.cities[road.origin]['lat']
            slon = self.cities[road.origin]['lon']
            dlat = self.cities[road.destination]['lat']
            dlon = self.cities[road.destination]['lon']
            if action_list and road in action_list:
                color = 'red'
                linewidth = 15
            else:
                color = 'lime'
                linewidth = 5
            ax.plot([slon , dlon], [slat, dlat], linewidth=linewidth, color=color, alpha=0.5)

# %% [markdown]
# Vamos a tener un diccionario donde esten todas las acciones como las del ejemplo de abajo, y luego un metodo para obtener esas acciones.
# 
# Para hacer el diccionario, si origen es igual a 0 declaramos esa ciudad como clave para el diccionario de accion. <br>
# La **Ciudad 0** puede viajar a **1 y a 5** <br>
# Ej: {Ciudad_0 : [viaja_1, viaja_5], Ciudad_1 : [viaja_0, viaja_5], Ciudad_2 : viaja_9,.......} <br>
# Ej con numeros: {0 : [1, 5], 1 : [0, 5], 2 : 9,....} <br>
# Cuando estemos en una ciudad, el estado actual será la clave del diccionario de acciones.
# 
# Por tanto el metodo **get_action** elegira un valor el cual se le pasara. Supongamos que quiero saber donde se puede viajar desde la ciudad 2, se le pasa al get_action la ciudad que queremos ver y este metodo nos devuelve la lista de posibles ciudades a las que podemos viajar. <br>

# %% [markdown]
# #### Class `Search`
# 
# The `Search` class is in abstract class that contains some attributes:
# - The `Problem` to solve.
# - The list of `open` nodes, i.e. nodes in the frontier, which data structure varies from the different algorithms.
# - The list of `closed` nodes to implement the graph search, which must be implemented using a `set` data structure.
# - The statistics from the algorithm, this is:
#     - The execution time (in ms) to obtain the solution.
#     - The cost of the solution.
#     - The number of generated nodes.
#     - The number of expanded nodes.
#     - The maximum number of nodes simultaneously stored in memory.
#     - The solution obtained (sequence of actions).
# 
# This class also provides some abstract methods:
# - `insert_node(self, node, node_list)`: Method to insert a node in the proper place in the list. May vary from one algorithm to another.
# - `extract_node(self, node_list)`: Method to extract a node from the open node list. Can vary from one data structure to another.
# - `is_empty(self, node_list)`: Method to check if the open node list is empty. Can vary from one data structure to another.
# 
# Methods you must add: 
# - `__init__(self, args)`: Constructor of the class, with the necessary arguments
# - `get_successors(self, node)`: this method implements the successors function and should return a list with all the valid `Node` successors of a given `Node`. You must implement this method.
# - `do_search(self)`: this method implements the graph search you have studied in class. It also provides some statistics of the search process.
# - A method that returns a human readable list of cities from a list of actions. It should be used to return a readable solution instead of the raw list of actions. 
# 
# Note that these methods have to be compatible with both informed and uninformed search. This is why you have `insert_node`, `extract_node` and `is_empty`: as you will need to use different data structures for informed and uninformed algorithms, just by implementing those methods you can make the general `Search` class agnostic to the data structure underlying. 

# %%
class Search(ABC):
    def __init__(self, problem):
        
        self.problem = problem

        self.frontier = []

        self.generated_nodes = 0

        # Lista de nodos expandidos
        self.explored = set()

        # Lista de nodos para llegar a la solucion
        self.solution = None

        # String de la solucion tras pasarlo de acciones a ciudades
        self.solution_string =  None

        # Coste de la solucion en km
        self.solution_cost = None

        # PROFUNDIDAD a la que se ha encontrado la solucion
        self.solution_depth = None

        # Numero de nodos expandidos
        self.expanded_nodes = None

        # Lista de nodos cerrados
        self.closed_nodes = None

        # Tiempo de ejecucion
        self.execution_time = None

    def __repr__(self):
        if self.solution is None:
            return 'No se ha encontrado solucion'
        else:
            # Formato de salida propio
            return f"""Recorrido de la solucion:
{self.solution_string}
Coste de la solucion: {self.solution_cost} km
Profundidad de la solucion: {self.solution_depth}
Numero de nodos expandidos: {self.expanded_nodes}
Nodos generados: {self.generated_nodes}
Lista de nodos expandidos: {self.explored}
Tiempo de ejecucion: {self.execution_time}
"""
# Formato de salida txt

#          return f"""Solution length: {self.solution_depth}
# Solution cost: {self.solution_cost}
# Solution: 
# {self.solution}
# City Path:
# {self.solution_string}
# Generated nodes: {self.generated_nodes}
# Expanded nodes: {self.expanded_nodes}
# Time elapsed: {self.execution_time}"""


         
    def __str__(self):
        if self.solution is None:
            return 'No se ha encontrado solucion'
        else:
            # Formato de salida propio
            return f"""Recorrido de la solucion:
{self.solution_string}
Coste de la solucion: {self.solution_cost} km
Profundidad de la solucion: {self.solution_depth}
Numero de nodos expandidos: {self.expanded_nodes}
Nodos generados: {self.generated_nodes}
Lista de nodos expandidos: {self.explored}
Tiempo de ejecucion: {self.execution_time}
"""
# Formato de salida txt

#          return f"""Solution length: {self.solution_depth}
# Solution cost: {self.solution_cost}
# Solution: 
# {self.solution}
# City Path:
# {self.solution_string}
# Generated nodes: {self.generated_nodes}
# Expanded nodes: {self.expanded_nodes}
# Time elapsed: {self.execution_time}"""




##################################################################################################
#                                   METODOS EXPANDIR Y BUSQUEDA                                  #
##################################################################################################

    # La función EXPANDIR crea nuevos nodos, rellenando sus campos y usa la función
    # SUCESOR del problema para crear los estados correspondientes.
    def get_successors(self, node):

        # Creamos una lista de nodos vacia
        successors = []
        # Recorremos la lista de acciones que se pueden realizar desde el nodo actual
        for action in self.problem.get_actions(node.state):
            # Guardamos el destino de la accion en una variable auxiliar para no tener que acceder al diccionario cada vez que la necesitemos
            new_state = action.destination
            # Creamos un nuevo nodo con el estado, el nodo padre, la accion, el coste y la profundidad
            new_node = Node(new_state, node, action, node.path_cost + action.distance, node.depth + 1)
            # Añadimos el nodo a la lista de nodos hijos
            successors.append(new_node)
            # Contamos el numero de nodos generados
            self.generated_nodes += 1
        # Devolvemos la lista de nodos hijos
        return successors
        

    def do_search(self):
        # Inicializamos el tiempo de ejecucion
        initial_time = perf_counter()

        # Creamos el nodo raiz
        initial_node = Node(self.problem.departure, None, None, 0, 0)

        self.generated_nodes += 1
        # Añadimos el nodo inicial a la frontera
        self.insert_node(initial_node)

        # Mientras la frontera no este vacia
        while self.frontier:
            # Sacamos un nodo de la frontera dependiendo del tipo de algoritmo
            node = self.extract_node()
            # Si el nodo es el objetivo
            if self.problem.comprueba_objetivo(node.state):
                # Creamos la solucion
                self.solution = self.get_solution(node)
                # Llamamos a la funcion que crea la cadena de la solucion
                self.solution_action2string()
                # Calculamos el coste de la solucion
                # self.solution_cost = self.calculate_solution_cost()
                self.solution_cost = node.path_cost
                # Calculamos la profundidad de la solucion
                # self.solution_depth = self.calculate_solution_depth()
                self.solution_depth = node.depth
                # Calculamos el numero de nodos expandidos
                self.expanded_nodes = len(self.explored)        
                final_time = perf_counter()
                self.execution_time = final_time - initial_time
                # Devolvemos la solucion
                return self.solution
            # Si el nodo no esta en la lista de expandidos
            if node.state not in self.explored:
                # Añadimos el nodo a la lista de expandidos
                self.explored.add(node.state)            
                # Añadimos los nodos hijos a la frontera
                # self.frontier.extend(self.get_successors(node))
                for child in self.get_successors(node):
                    self.insert_node(child)
                
                

        # Si la frontera esta vacia entonces no hay solucion
        # raise Exception('No solution found')
        return None

###################################################################################
#                           ZONA SOLUCIONES                                       #
###################################################################################


    # Funcion que pasa la lista de acciones a string para poder imprimirlo
    def solution_action2string(self):
        action_string = ''
        # # No deberia llegar aqui pero por si acaso
        # if (self.solution is None):
        #     raise Exception('Solucion vacia')
        # else:
        # print('¿Como quieres que se muestre la solucion?')
        # print('1. Numero de la ciudad')
        # print('2. Nombre de la ciudad')
        # input('Introduce el numero de la opcion: ')
        # if (input == 1):
        #     for action in self.solution:
        #         # Mostrar ciudades con numeros
        #         action_string += str(action.origin) + ' -> ' + str(action.destination) + '  '
        
        # else:
            # Mostrar ciudades con nombres
        for action in self.solution:
            # action_string += '(' + str(self.problem.cities[action.origin]['name']) + ' -> ' + str(self.problem.cities[action.destination]['name']) + ')\n'
            action_string += '(' + str(self.problem.cities[action.origin]['name']) + ' -> ' + str(self.problem.cities[action.destination]['name']) + ')  '           
            # action_string += str(self.problem.cities[action.origin]['name']) + ' ->   '        
        # # Añadimos la ultima ciudad
        # action_string += str(self.problem.cities[self.solution[-1].destination]['name'])
        self.solution_string = action_string


    #  Funciones de calculo de la solucion 
    def calculate_solution_cost(self):
        # Sumar todas las distancias de las acciones
        return sum([action.distance for action in self.solution])

    def calculate_solution_depth(self):
        # Calcular numero de acciones a realizar
        return len(self.solution) - 1  

    def get_solution(self, node):
        solution = []
        while node.parent is not None:
            solution.append(node.action)
            node = node.parent
        solution.reverse()
        return solution

###################################################################################

    @abstractmethod
    def insert_node(self, node):
        pass

    @abstractmethod
    def extract_node(self):
        pass

    @abstractmethod
    def is_empty(self):
        pass


# %% [markdown]
# #### Uninformed Search: `DepthFirst` and `BreadthFirst`
# 
# These two classes also inherit from `Search` and will implement the depth first and breadth first. As explained before, if you have implemented `get_successors(self, node)` and `do_search(self)` properly, you just have to implement the `insert_node(self, node, node_list)`, `extract_node` and `is_empty` functions. 

# %%
class DepthFirstSearch(Search):
    def __init__(self, parent_args):
        super().__init__(parent_args)
        

    def __repr__(self):
        return f'DepthFirstSearch:\n{super().__repr__()}'

    def __str__(self):
        return f'DepthFirstSearch:\n{super().__str__()}'

    def insert_node(self, node):
        self.frontier.append(node)

    def extract_node(self):
        return self.frontier.pop()

    def is_empty(self):
        return len(self.frontier) != 0


# %%
class BreadthFirst(Search):
    def __init__(self, parent_args):
        super().__init__(parent_args)
        

    def __repr__(self):
        return f'BreadthFirst:\n{super().__repr__()}'

    def __str__(self):
        return f'BreadthFirst:\n{super().__str__()}'

    def insert_node(self, node):
        self.frontier.append(node)

    # El unico cambio es que ahora sacamos el primer elemento de la lista, esta es la unica diferencia con el DFS
    def extract_node(self):
        return self.frontier.pop(0)
    
    def is_empty(self):
        return len(self.frontier) != 0
    

# %% [markdown]
# #### Informed Search: `BestFirst` and `AStar`
# 
# These two classes also inherit from `Search` and will implement the best first and $A^*$ search strategies, respectively. 
# 
# The main difference between these three algorithms is the way in which the cost function for a specific node ($f(n) = g(n) + h(n)$) is computed. Assuming that $g(n)$ is the real accumulated cost from the **initial state** to `n.state` and that $h(n)$ is the heuristic cost from `n.state` state to the **goal state**, $f(n)$ is computed as:
# 
# - Best First: $f(n) = h(n)$
# - A$^*$: $f(n) = g(n) + h(n)$
# 
# As before, once the `get_successors(self,node)` and `do_search(self)` methods have been implemented in the parent class, we have to implement the `insert_node(self, node)` method, which will insert the `node` into the `self.open` list of nodes according to the corresponding values of the cost function, as well as the `extract_node` and `is_empty` methods.
# 
# You also have to implement a new `__init__(self, args)` constructor so that you can expand the behavior of the informed search algorithms with a `Heuristic` and any other methods you need.
# 
# It is greatly encouraged that you use the [Priority Queue](https://docs.python.org/3/library/queue.html#queue.PriorityQueue) structure for the informed search, as it will be an efficient structure to have your nodes ordered, rather than having to sort the list every single time. 

# %%
class BestFirst(Search):
    
    def __init__(self, parent_args, heuristica='geodesic'):
        # Calling the constructor of the parent class
        # with its corresponding arguments
        super().__init__(parent_args)
        # self.frontier = PriorityQueue()
        
        # Funcion heuristica
        if heuristica == 'geodesic':
            self.heuristic = GeodesicHeuristic(self.problem)
        elif heuristica == 'manhattan':
            self.heuristic = ManhattanHeuristic(self.problem)
        elif heuristica == 'euclidean':
            self.heuristic = EuclideanHeuristic(self.problem)
        elif heuristica == 'chebyshev':
            self.heuristic = ChebyshevHeuristic(self.problem)
        else:
            raise Exception('Heuristica no encontrada')
        


    def __repr__(self):
        return f'BestFirst:\n{super().__repr__()}'

    def __str__(self):
        return f'BestFirst:\n{super().__str__()}'

    # Metemos el nodo a la frontera en orden segun la funcion heuristica
    def insert_node(self, node):
        # Version con PriorityQueue
        # self.frontier.put((self.heuristic.get_hcost(node), node))


        # VERSION 1 sin PriorityQueue
        # if len(self.frontier) == 0:
        #     self.frontier.append(node)
        # else:
        #     # Recorremos la lista para introducir el nodo en orden
        #     for i in range(len(self.frontier)):
        #         # Si el nodo a introducir tiene menor coste que el nodo de la lista
        #         if self.heuristic.get_hcost(node) < self.heuristic.get_hcost(self.frontier[i]):
        #             # Introducimos el nodo en la posicion i
        #             self.frontier.insert(i, node)
        #             # Salimos del bucle
        #             break
        #         elif self.heuristic.get_hcost(node) == self.heuristic.get_hcost(self.frontier[i]):
        #             # Si el nodo a introducir tiene el mismo coste que el nodo de la lista
        #             # Introducimos el nodo en la posicion i + 1
        #             self.frontier.insert(i + 1, node)
        #             # Salimos del bucle
        #             break

        #         # Si hemos llegado al final de la lista
        #         elif i == len(self.frontier) - 1:
        #                 # Introducimos el nodo al final
        #                 self.frontier.append(node)
        #                 # Salimos del bucle
        #                 break

        # VERSION 2 sin PriorityQueue Igual que la primera pero guardando el valor de la heuristica en forma de tupla como PriorityQueue
        node_heuristic_cost = self.heuristic.get_hcost(node)
        if len(self.frontier) == 0:
            self.frontier.append((node_heuristic_cost, node))
        else:
            # Recorremos la lista para introducir el nodo en orden
            for i in range(len(self.frontier)):
                # Si el nodo a introducir tiene menor coste que el nodo de la lista
                if node_heuristic_cost < self.frontier[i][0]:
                    # Introducimos el nodo en la posicion i
                    self.frontier.insert(i, (node_heuristic_cost, node))
                    # Salimos del bucle
                    break
                elif node_heuristic_cost == self.frontier[i][0]:
                    # Si el nodo a introducir tiene el mismo coste que el nodo de la lista
                    # Introducimos el nodo en la posicion i + 1
                    self.frontier.insert(i + 1, (node_heuristic_cost, node))
                    # Salimos del bucle
                    break

                # Si hemos llegado al final de la lista
                elif i == len(self.frontier) - 1:
                        # Introducimos el nodo al final
                        self.frontier.append((node_heuristic_cost, node))
                        # Salimos del bucle
                        break
            



    # Extraemos el primer nodo de la frontera
    def extract_node(self):
        # Cogemos el valor en la posicion 1 puesto que en la tupla declaramos
        # primero el valor de la heuristica y despues el nodo.
        # return self.frontier.get()[1]
        # v1 return self.frontier.pop(0)
        return self.frontier.pop(0)[1]   
    
    # Comprobamos si la frontera esta vacia
    def is_empty(self):
        return len(self.frontier) != 0



# %%
class AStar(Search):
    def __init__(self, parent_args, heuristica='geodesic'):
        # Calling the constructor of the parent class
        # with its corresponding arguments
        super().__init__(parent_args)

        # self.frontier = PriorityQueue()
        
        # Funcion heuristica
        if heuristica == 'geodesic':
            self.heuristic = GeodesicHeuristic(self.problem)
        elif heuristica == 'manhattan':
            self.heuristic = ManhattanHeuristic(self.problem)
        elif heuristica == 'euclidean':
            self.heuristic = EuclideanHeuristic(self.problem)
        elif heuristica == 'chebyshev':
            self.heuristic = ChebyshevHeuristic(self.problem)
        else:
            raise Exception('Heuristica no encontrada')
        


    def __repr__(self):
        return f'AStar:\n{super().__repr__()}'

    def __str__(self):
        return f'AStar:\n{super().__str__()}'
        

    def insert_node(self, node):
        # self.frontier.put((self.heuristic.get_hcost(node) + node.path_cost, node))
        
        # if len(self.frontier) == 0:
        #     self.frontier.append(node)
        # else:
        # # Recorremos la lista para introducir el nodo en orden
        #     for i in range(len(self.frontier)):
        #         # Si el nodo a introducir tiene menor coste que el nodo de la lista
        #         if self.heuristic.get_hcost(node) + node.path_cost < self.heuristic.get_hcost(self.frontier[i]) + self.frontier[i].path_cost:
        #             # Introducimos el nodo en la posicion i
        #             self.frontier.insert(i, node)
        #             # Salimos del bucle
        #             break
        #         elif self.heuristic.get_hcost(node) + node.path_cost == self.heuristic.get_hcost(self.frontier[i]) + self.frontier[i].path_cost:
        #             # Si el nodo a introducir tiene el mismo coste que el nodo de la lista
        #             # Introducimos el nodo en la posicion i + 1
        #             self.frontier.insert(i + 1, node)
        #             # Salimos del bucle
        #             break

        #         # Si hemos llegado al final de la lista
        #         elif i == len(self.frontier) - 1:
        #                 # Introducimos el nodo al final
        #                 self.frontier.append(node)
        #                 # Salimos del bucle
        #                 break

        # VERSION 2 Igual que la primera pero guardando el valor de la heuristica en forma de tupla como PriorityQueue
        node_heusistic_cost = self.heuristic.get_hcost(node) + node.path_cost
        if len(self.frontier) == 0:
            self.frontier.append((node_heusistic_cost, node))
        else:
            # Recorremos la lista para introducir el nodo en orden
            for i in range(len(self.frontier)):
                
                # Si el nodo a introducir tiene menor coste que el nodo de la lista
                if node_heusistic_cost < self.frontier[i][0]:
                    # Introducimos el nodo en la posicion i
                    self.frontier.insert(i, (node_heusistic_cost, node))
                    # Salimos del bucle
                    break
                elif node_heusistic_cost == self.frontier[i][0]:
                    # Si el nodo a introducir tiene el mismo coste que el nodo de la lista
                    # Introducimos el nodo en la posicion i + 1
                    self.frontier.insert(i + 1, (node_heusistic_cost, node))
                    # Salimos del bucle
                    break

                # Si hemos llegado al final de la lista
                elif i == len(self.frontier) - 1:
                        # Introducimos el nodo al final
                        self.frontier.append((node_heusistic_cost, node))
                        # Salimos del bucle
                        break
            
            

    def extract_node(self):
        return self.frontier.pop(0)[1]
        # return self.frontier.get()[1]

    def is_empty(self):
        return len(self.frontier) != 0

# %% [markdown]
# #### Heuristics
# 
# An informed search must have an heuristic, and the way to implement is by creating a class for each heuristic. The different classes must inherit from the abstract class `Heuristic` provided here. They must implement the `get_hcost(self, node)` method to return the heuristic of a node. They can also implement a constructor where some information about the problem is given to compute that heuristic.

# %%
class Heuristic(ABC):   
    @abstractmethod
    def get_hcost(self, node):
        pass

# %%

class EuclideanHeuristic(Heuristic):
    def __init__(self, problem):
        self.problem = problem

    def get_hcost(self, node):
        euclidean_distance = math.sqrt((self.problem.cities[node.state]['lat'] - self.problem.cities[self.problem.goal]['lat'])**2 + (self.problem.cities[node.state]['lon'] - self.problem.cities[self.problem.goal]['lon'])**2)
        return euclidean_distance

# %%
# Calcular la distancia geodesica entre dos puntos
class GeodesicHeuristic(Heuristic):
    def __init__(self, problem):
        self.problem = problem
        
    def get_hcost(self, node):
        # Calculamos la distancia geodesica entre dos puntos
        geodesic_distance = geodesic((self.problem.cities[node.state]['lat'], self.problem.cities[node.state]['lon']), (self.problem.cities[self.problem.goal]['lat'], self.problem.cities[self.problem.goal]['lon'])).km
        return geodesic_distance

# %%
class ManhattanHeuristic(Heuristic):
    def __init__(self, problem):
        self.problem = problem

    def get_hcost(self, node):
        manhattan_distance = abs(self.problem.cities[node.state]['lat'] - self.problem.cities[self.problem.goal]['lat']) + abs(self.problem.cities[node.state]['lon'] - self.problem.cities[self.problem.goal]['lon'])
        return manhattan_distance

# %%
class ChebyshevHeuristic(Heuristic):
    def __init__(self, problem):
        self.problem = problem

    def get_hcost(self, node):
        chebyshev_distance = max(abs(self.problem.cities[node.state]['lat'] - self.problem.cities[self.problem.goal]['lat']), abs(self.problem.cities[node.state]['lon'] - self.problem.cities[self.problem.goal]['lon']))
        return chebyshev_distance

# %% [markdown]
# As an example, the optimistic heuristic is given below. Take into account that you can add information to your heuristic by adding elements in the constructor of the class.

# %%
class OptimisticHeuristic(Heuristic):
    def __init__(self, info):
        self.info = info
    
    def get_hcost(self, node):
        return 0

# %% [markdown]
# PRUEBAS

# %%
###################################################################################
#                             PRUEBAS INDIVIDUALES                                # 
# ###################################################################################
# problem = Problem('Json/problem4f.json')


# # DEPTH FIRST SEARCH
# depth_first = DepthFirstSearch(problem)
# depth_first.do_search()
# print(depth_first)

# # BREADTH FIRST SEARCH
# breadth_first = BreadthFirst(problem)
# breadth_first.do_search()
# print(breadth_first)

# # BEST FIRST SEARCH

# best_first = BestFirst(problem, 'geodesic')
# best_first.do_search()
# print(best_first)

# # A* SEARCH
# a_star = AStar(problem, 'geodesic')
# a_star.do_search()
# print(a_star)

# %%
###################################################################################
#                             TODAS LAS PRUEBAS                                   #
###################################################################################

# import os
 

# # Cargamos todos los archivos de la carpeta Json para que devuelva resultados de forma iterativa
# for file in os.listdir('Json'):
#     if file.endswith('.json'):
#         problema = Problem('Json/' + file)
#         print('\n\nPROBLEMA JSON:\t\t\t' + file)
#         print('Solucion: ')
        
#         # DEPTH FIRST SEARCH
#         DFS = DepthFirstSearch(problema)
#         DFS.do_search()
#         print(DFS)

#         # BREADTH FIRST SEARCH
#         BFS = BreadthFirst(problema)
#         BFS.do_search()
#         print(BFS)
        
#         # BEST FIRST SEARCH
#         best_first = BestFirst(problema, 'geodesic')
#         best_first.do_search()
#         print(best_first)

#         # A* SEARCH
#         a_star = AStar(problema, 'geodesic')
#         a_star.do_search()
#         print(a_star)     
#         print('\n------------------------------------------------------------')

# %%
###################################################################################
#                        PRUEBA ENTRE DIFERENTES HEURISTICAS                      # 
# ###################################################################################
# problem = Problem('Json/problem5f.json')


# # BEST FIRST SEARCH
# for heuristic in ['euclidean', 'geodesic', 'manhattan', 'chebyshev']:
#     print("Heuristica: " + heuristic)
#     best_first = BestFirst(problem, heuristic)
#     best_first.do_search()
#     print(best_first)

# # A* SEARCH
# for heuristic in ['euclidean', 'geodesic', 'manhattan', 'chebyshev']:
#     print("Heuristica: " + heuristic)
#     a_star = AStar(problem, heuristic)
#     a_star.do_search()
#     print(a_star)

# %%
###################################################################################
#                         DIFERENCIAS ENTRE HEURISTICAS                           #
# ###################################################################################
# import os

# # Cargamos todos los archivos de la carpeta Json para que devuelva resultados de forma iterativa
# for file in os.listdir('Json'):
#     if file.endswith('.json'):
#         problema = Problem('Json/' + file)
#         print('\n\nPROBLEMA JSON:\t\t\t' + file)
#         print('Solucion: ')
        
#         # BEST FIRST SEARCH
#         for heuristic in ['euclidean', 'geodesic', 'manhattan', 'chebyshev', 'optimistic']:
#             best_first = BestFirst(problema, heuristic)
#             best_first.do_search()
#             print(best_first)

#         # A* SEARCH
#         for heuristic in ['euclidean', 'geodesic', 'manhattan', 'chebyshev', 'optimistic']:
#             a_star = AStar(problema)
#             a_star.do_search()
#             print(a_star)     
#         print('\n------------------------------------------------------------')

# %%
###################################################################################
#                     PRUEBA COMPROBADA CON SOLUCION                              #
# ###################################################################################

# p1 = Problem('Json/problem2.json')

# # BEST FIRST SEARCH
# best_first = BestFirst(problem)
# best_first.do_search()
# print(best_first)

# # Comparamos los resultados con la solucion en el txt
# with open('Soluciones/problem2-best-first-solution.txt', 'r') as file:
#     data = file.read().replace('\n', '')
#     # data = file.read().splitlines()
#     # Buscamos en data el coste de la solucion definida como Solution cost:
#     coste = data[data.find('Solution cost:')]
#     # print('Coste de la solucion: ' + coste)
# # Comprobamos si la solucion es correcta
# if (best_first.solution_cost == coste):
#     print('Solucion correcta')
# else:
#     print("Solucion incorrecta: ")


# %% [markdown]
# ### 4.3 Study and improvement of the algorithms
# Once the algorithms have been implemented, you must study their performance. In order to do that,
# you must compare the quality of the solutions obtained, as well as the number of expanded nodes for
# instances of different sizes. Factors as the maximum size of problem that can be solved (without memory
# overflow), or the effect of using more complex scenarios, are also important. Moreover, you can propose
# alternative implementations that increase the efficiency of the algorithms.

# %% [markdown]
# 

# %% [markdown]
# ### 4.4 Report
# Besides the notebook containing the implementation, the assignment consists in the elaboration of a report, which will have a later deadline, but you should be developing when your code starts solving problems
# correctly. 
# 
# In particular, among other issues that the student deems of interest to discuss, it should include:
# 
# - A brief description of the problem, a description of the implementation, the performance evaluation, and the description of improvements if they exist. 
# - The formalization of the problem.
# - For informed search algorithms one (or several) heuristic functions must be provided. Apart from their description and motivation, an analysis should be included indicating whether the proposed heuristic is considered admissible and/or consistent.
# - The study of performance of implemented algorithms should be based on testing the algorithms over several instances, presenting tables or graphics that summarize results (do not include screenshots).
# 
# The memory must not include figures with source code, unless this is necessary to explain some key concept (data structures, improvements in efficiency, etc). In such cases, you are allowed to include
# properly formatted pseudocode.

# %% [markdown]
# ## 5. Submission and evaluation
# The work must be made in pairs, although in some exceptional cases you can present it individually. The deadline for submission is 6th November, 2022. Interviews and evaluations will be in the following week. 
# 
# You must work on your notebook on the Datalore project, as the day of the deadline it will run some automated tests and collect the notebooks in the state they are. No changes will be allowed after the deadline. 
# 
# Some considerations related to the evaluation:
# - This is 30% of the lab grade. Lab2 (70%) needs the resolution of this part. Late submissions
# (together with lab2) or failed assignments will be evaluated globally but with a penalization of factor
# 0.9, as the students can only get 90% of the lab grade.
# - Attendance with performance to the lab not only will provide half of the participation grade, but
# it will also be the best foundation for successful resolution of the labs.
# - The assignment will be evaluated during an individual interview with the professors. Dates for the
# interviews will be published with some advance in Campus Virtual.
# - We will provide a set of preliminary test cases (several maps and queries) that must be correctly
# solved. Otherwise, the assignment will be considered as failed.
# - In order to get a mark in the assignment you will have to answer, individually, a set of basic
# questions about the code organization.
# - In the non-continuous evaluation we will require the implementation of the same strategies plus
# these extra two: Depth-limited search and Iterative deepening search.

# %%



