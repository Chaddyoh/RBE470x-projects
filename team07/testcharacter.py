# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import heapq
import math

class Enum():
    TRAVELING = 0
    FLEEING = 1

class PriorityQueue():
    
    def __init__(self):
        self.elements = []

    def empty(self):
        """
        Returns True if the queue is empty, False otherwise.
        """
        return len(self.elements) == 0
    
    def put(self, element, priority):
        """
        Puts an element in the queue.
        :param element  [any type]     The element.
        :param priority [int or float] The priority.
        """
        for i in range(0, len(self.elements)):
            it = self.elements[i]
            if (it[1] == element):
                if (it[0] > priority):
                    self.elements[i] = (priority, element)
                    heapq.heapify(self.elements)
                return
        heapq.heappush(self.elements, (priority, element))

    def get(self):
        """
        Returns the element with the top priority.
        """
        return heapq.heappop(self.elements)[1]
    
    def get_queue(self):
        """
        Returns the content of the queue as a list.
	"""
        return self.elements
    

class TestCharacter(CharacterEntity):
    state = Enum.TRAVELING
        
    def locate_exit(self, wrld) -> tuple: # Returns X,Y tuple for exit
        for x_coordinate in range(wrld.width()):
            for y_coordinate in range(wrld.height()):
                if wrld.exit_at(x_coordinate, y_coordinate):
                    return (x_coordinate, y_coordinate)   
                         
    def get_neighbors(self, wrld, current) -> list[tuple]: # Returns a list of tuples of the surrounding empty nodes. Assumes the exit node is empty
        neighbors = []
        for dx in [-1,0,1]:
            if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
                for dy in [-1,0,1]:
                    if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                        if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy):
                            neighbors.append((current[0]+dx, current[1]+dy))
        return neighbors    
                
    def heuristic(self, point1, point2) -> float:
        euclidean_dist = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        return euclidean_dist
    
    def plan_path(self, wrld, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            for next in self.get_neighbors(wrld, current):
                self.set_cell_color(next[0], next[1], Fore.WHITE + Back.MAGENTA)
                new_cost = cost_so_far[current] + 1 # the cost to each adjacent cell is 1 # graph.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

            if current == goal:
                break
    
        path = [goal]
        previous_node = goal
        while not previous_node == start:
            next_node = came_from[previous_node]
            path.append(next_node)
            previous_node = next_node
        
        path.reverse()
        return path
    
    def color_path(self, path): 
        for coord in path: 
            self.set_cell_color(coord[0], coord[1], Fore.BLUE + Back.YELLOW)

    def next_step(self, path):
        current_node = path[0]
        next_node = path[1]

        dx = next_node[0] - current_node[0]
        dy = next_node[1] - current_node[1]
        
        self.move(dx, dy)

    def check_for_monster(self, wrld, current): 
        neighbors = []
        for dx in [-3,0,3]:
            if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
                for dy in [-3,0,3]:
                    if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                        if wrld.monsters_at(current[0]+dx, current[1]+dy):
                            return (current[0]+dx, current[1]+dy)
        return None 
    
    def avoid_monster(self, monster_pos): # Ex (-3, 1)
        # expectimax stuff here
        self.next_step()

    def do(self, wrld):
        # Your code here
        exit = self.locate_exit(wrld)
        start = (self.x, self.y)


        match self.state:
            case Enum.TRAVELING:
                monster_loc = self.check_for_monster(wrld, (self.x, self.y))
                if monster_loc: 
                    self.avoid_monster(monster_loc)
                    self.state = Enum.FLEEING
                else: 
                    path = self.plan_path(wrld, start, exit)
                    self.color_path(path)
                    if len(path) > 1:
                        self.next_step(path)
            case Enum.FLEEING:
                pass
        
               
