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
                        if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy):
                            neighbors.append((current[0]+dx, current[1]+dy))
        return neighbors    
                
    def heuristic(self, point1, point2) -> float:
        euclidean_dist = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        return euclidean_dist
    
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
    
    def color_path(self, path) -> None: 
        for coord in path: 
            self.set_cell_color(coord[0], coord[1], Fore.BLUE + Back.YELLOW)

    def next_step(self, path) -> None:
        current_node = path[0]
        next_node = path[1]

        dx = next_node[0] - current_node[0]
        dy = next_node[1] - current_node[1]
        
        self.move(dx, dy)

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
                        # if monsters:
                        #     print("Monster detected at: ", (current[0]+dx, current[1]+dy))
                        #     # return (current[0]+dx, current[1]+dy)
        return monsters_list 
    
    
    def avoid_monster(self, monster_pos, wrld): # Ex (-3, 1)
        # expectimax stuff here
        actions = [(1,0), (1,1), (0,1), (-1,0), (-1, -1), (0, -1), (-1, 1), (1,-1), (0,0)] # all player moves
        depth_counter = 0
        # add the option of not moving 

        def Expectimax_Search(state): # returns an action
            global depth_counter
            # return arg maxa∈Actions(state)Exp-value(Result(state,a))
            max_so_far = -math.inf
            best_action = (0,0)
            print("currnt loc: ", self.x, self.y)
            for a in actions: 
                depth_counter = 0
                v = Exp_value(result(state, a))
                print(depth_counter)
                print("(in top search) expected val for action ", a, " is ", v)
                if v > max_so_far: 
                    max_so_far = v
                    best_action = a
            self.set_cell_color(state[0], state[1], Fore.BLUE + Back.BLUE)
            print("best action is: ", best_action)
            return best_action

        def Exp_value(state): #returns a utility value 
            global depth_counter
            depth_counter += 1 
            monster_list = terminal_test(state, wrld)
            if monster_list:  return utility(state, monster_list)
            v = 0
            for a in actions: #Actions of state will be list of tuples # used to be actions[state]
                p = Probability(a)
                v = v + p * Max_value(result(state, a))
            return v

        def Max_value(state): # returns a utility value
            global depth_counter
            depth_counter += 1 
            monster_list = terminal_test(state, wrld)
            if monster_list: return utility(state, monster_list)
            v = -math.inf
            for a in actions:
                v = max(v, Exp_value(result(state,a)))
                print("expected val for action ", a, " is ", v)
            print("max val: ", v)
            return v
        
        def Probability(action, monsterlist):
            """Return a probability that the monster will take this action"""
            # TODO: monster has higher probability for action towards player character 
            for monster in monsterlist:
                if monster.name == "stupid":
                    return 1.0/8
                elif monster.name == "aggressive":
                    return 1     
                   
        def result(location, action) -> tuple: 
            """Returns the new location after taking an action"""
            new_x = location[0] + action[0]
            new_y = location[1] + action[1]
            
            print("result: ", (new_x, new_y))
            return (new_x, new_y)
        
        def terminal_test(state, wrld): 
            """ True if we have successfully avoided the monster. Can stop fleeing"""
            # if self.check_for_monster(wrld, (self.x, self.y)) == False:
            #     print("Monster Avoided!! Going back to path planning...")
            return self.check_for_monster(wrld, (self.x, self.y))

        def utility(state, monster_list):
            # TODO: account for the distance between this state and the exit 
            alpha = 0.05
            utility = 0
            if state[0] > 0 and state[0] < wrld.width() and state[1] > 0 and state[1] < wrld.height():
                if wrld.empty_at(state[0], state[1]):
                    for monster in monster_list:
                        monster_loc = (monster.x, monster.y)
                        monster_distance = self.heuristic(monster_loc, state)
                        utility += monster_distance ** 2
                    exit_distance = len(self.plan_path(wrld, state, self.exit))
                    utility -= alpha * exit_distance
                else:
                    utility = 0
            else:
                utility = 0
            return utility
            
        best_action = Expectimax_Search((self.x, self.y))
        self.move(best_action[0], best_action[1])
    
    
                
                           
    def do(self, wrld):
        # Your code here
        self.exit = self.locate_exit(wrld)
        start = (self.x, self.y)
        self.longest_dist = self.heuristic((0,0), self.exit)
        monster_loc = None

        if self.check_for_monster(wrld, (self.x, self.y)):
            first_seen_monster = self.check_for_monster(wrld, (self.x, self.y))[0]
            monster_loc = (first_seen_monster.x, first_seen_monster.y)

        match self.state:
            case Enum.TRAVELING:
                if monster_loc: 
                    self.avoid_monster(monster_loc, wrld)
                    print("Begin fleeing")
                    self.state = Enum.FLEEING
                else: 
                    path = self.plan_path(wrld, start, self.exit)
                    self.color_path(path)
                    self.next_step(path)
            case Enum.FLEEING:
                if monster_loc: 
                    self.avoid_monster(monster_loc, wrld)
                else:
                    path = self.plan_path(wrld, start, self.exit)
                    self.color_path(path)
                    self.next_step(path)
                    self.state = Enum.TRAVELING
                       

        
               
