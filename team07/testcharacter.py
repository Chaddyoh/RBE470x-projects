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
    WAITING = 2
    TRAPPED = 3

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
    max_depth = 10
    monsters = []
        
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
                        if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy) or wrld.wall_at(current[0]+dx, current[1]+dy):
                            neighbors.append((current[0]+dx, current[1]+dy))
        return neighbors    
                
    def heuristic(self, point1, point2) -> float:
        euclidean_dist = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        return euclidean_dist
    
    def cost(self, wrld, location): 
        if wrld.empty_at(location[0], location[1]) or wrld.exit_at(location[0], location[1]): 
            return 1
        elif wrld.wall_at(location[0], location[1]): 
            return 13
    
    def plan_path(self, wrld, start, goal) -> list[tuple]:
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
                new_cost = cost_so_far[current] + self.cost(wrld, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

            if current == goal:
                break

        try:
            path = [goal]
            previous_node = goal
            while not previous_node == start:
                next_node = came_from[previous_node]
                path.append(next_node)
                previous_node = next_node
            
            path.reverse()
        except:
            path = []
            
        return path
    
    def color_path(self, path) -> None: 
        for coord in path: 
            self.set_cell_color(coord[0], coord[1], Fore.BLUE + Back.YELLOW)

    def next_step(self, wrld, path) -> None:
        current_node = path[0]
        next_node = path[1]

        dx = next_node[0] - current_node[0]
        dy = next_node[1] - current_node[1]
        
        if wrld.empty_at(self.x + dx, self.y + dy):
            self.move(dx, dy)
        else: 
            self.place_bomb()

    def check_for_monster(self, wrld, current) -> tuple: 
        global monsters
        monsters_list = []
        for dx in [-3,-2,-1,0,1,2,3]:
            if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
                for dy in [-3,-2,-1,0,1,2,3]:
                    if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                        monster_square = wrld.monsters_at(current[0]+dx, current[1]+dy)
                        if monster_square:
                            monsters_list += monster_square
        return monsters_list 
    
    def monster_range(self, wrld, state, monster):
        if monster.name == "selfpreserving":
            for dx in [-1,0,1]:
                search_x = monster.x + dx
                if (search_x>=0) and (search_x<wrld.width()):
                    for dy in [-1,0,1]:
                        search_y = monster.y + dy
                        if (search_y>=0) and (search_y<wrld.width()):
                            if state == (search_x, search_y):
                                return True
        elif monster.name == "aggressive":
            for dx in [-2,-1,0,1,2]:
                search_x = monster.x + dx
                if (search_x>=0) and (search_x<wrld.width()):
                    for dy in [-2,-1,0,1,2]:
                        search_y = monster.y + dy
                        if (search_y>=0) and (search_y<wrld.width()):
                            if state == (search_x, search_y):
                                return True
        return False

    def is_valid_space(self, wrld, loc):
        if loc[0] > 0 and loc[0] < wrld.width() and loc[1] > 0 and loc[1] < wrld.height(): # if state location is within the map 
            return wrld.empty_at(loc[0], loc[1]) # Code to check if empty except for wall: wrld.exit_at(loc[0], loc[1]) or wrld.bomb_at(loc[0], loc[1]) or wrld.explosion_at(loc[0], loc[1]) or wrld.monsters_at(loc[0], loc[1]) or wrld.characters_at(loc[0], loc[1]) 
               
        return False

    def avoid_monster(self, wrld): # Ex (-3, 1)
        # expectimax stuff here
        actions = [(1,0), (1,1), (0,1), (-1,0), (-1, -1), (0, -1), (-1, 1), (1,-1), (0,0)] # all player moves
        self.depth_counter = 0

        def Expectimax_Search(state): # returns an action
            max_so_far = -math.inf
            best_action = (0,0)
            for a in actions: 
                if self.is_valid_space(wrld, result(state, a)): 
                    self.depth_counter = 0
                    v = Exp_value(result(state, a))
                    if v > max_so_far: 
                        max_so_far = v
                        best_action = a
            self.set_cell_color(state[0], state[1], Fore.BLUE + Back.BLUE)
            return best_action

        def Exp_value(state): #returns a utility value 
            monster_list = terminal_test(state, wrld)
            if (not monster_list) or (self.depth_counter > self.max_depth): 
                return utility(state, monster_list)
            v = 0
            self.depth_counter += 1 
            for a in actions: #Actions of state will be list of tuples # used to be actions[state]
                if self.is_valid_space(wrld, result(state, a)): 
                    p = Probability(result(state, a), monster_list)
                    v = v + p * Max_value(result(state, a))
            return v

        def Max_value(state): # returns a utility value
            monster_list = terminal_test(state, wrld)
            if (not monster_list) or (self.depth_counter > self.max_depth): return utility(state, monster_list)
            v = -math.inf
            self.depth_counter += 1 
            for a in actions:
                if self.is_valid_space(wrld, result(state, a)): 
                    v = max(v, Exp_value(result(state,a)))
            return v
        
        def Probability(state, monsterlist):
            """Return a probability that the monster will take this action"""
            for monster in monsterlist:
                if monster.name == "stupid":
                    return 1.0/8 
                else:
                    return 1
                    # if self.monster_range(wrld, state, monster):
                    #     return 1
                    # else:
                    #     return 1.0/8
                   
        def result(location, action) -> tuple: 
            """Returns the new location after taking an action"""
            new_x = location[0] + action[0]
            new_y = location[1] + action[1]
            return (new_x, new_y)
        
        def terminal_test(state, wrld): 
            """ True if we have successfully avoided the monster. Can stop fleeing"""
            return self.check_for_monster(wrld, (self.x, self.y))

        def utility(state, monster_list):
            alpha_exit = 0.025
            utility = 0
            if self.is_valid_space(wrld, state):
                for monster in monster_list:
                    monster_loc = (monster.x, monster.y)
                    monster_distance = self.heuristic(monster_loc, state)
                    utility += monster_distance ** 2
                exit_distance = len(self.plan_path(wrld, state, self.exit))
                utility -= alpha_exit * exit_distance
            else:
                utility = 0
            return utility
            
        best_action = Expectimax_Search((self.x, self.y))
        self.move(best_action[0], best_action[1])
    
    def is_trapped(self, wrld, path):
        for cell in path:
            if wrld.wall_at(cell[0], cell[1]):
                return True
        return False

                
                           
    def do(self, wrld):
        # Your code here
        self.exit = self.locate_exit(wrld)
        start = (self.x, self.y)
        monster_loc = None

        if self.check_for_monster(wrld, (self.x, self.y)):
            first_seen_monster = self.check_for_monster(wrld, (self.x, self.y))[0]
            monster_loc = (first_seen_monster.x, first_seen_monster.y)

        path = self.plan_path(wrld, start, self.exit)
        trapped = self.is_trapped(wrld, path)

        match self.state:
            case Enum.TRAVELING:
                if monster_loc: 
                    self.avoid_monster(wrld)
                    self.state = Enum.FLEEING
                elif trapped:
                    self.color_path(path)
                    self.next_step(wrld, path)
                    print("help im trapped")
                else: 
                    self.color_path(path)
                    self.next_step(wrld, path)
            case Enum.FLEEING:
                if monster_loc: 
                    self.avoid_monster(wrld)
                elif trapped: 
                    self.color_path(path)
                    print("help im trapped")
                else:
                    self.color_path(path)
                    self.next_step(wrld, path)
                    self.state = Enum.TRAVELING                    
            case Enum.WAITING: 
                pass

                       

        
               
