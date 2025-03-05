# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from colorama import Fore, Back
import heapq
import math

class Enum():
    TRAVELING = 0
    FLEEING = 1
    BOMBING = 2
    WAITING = 3

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
    timestep = 0
    monsters = []
    weights = [-1, -1, 1, -1, -1, -1, -1, -1]
    bomb_loc = None
    bomb_placed_time = 0
        
    def locate_exit(self, wrld) -> tuple: # Returns X,Y tuple for exit
        for x_coordinate in range(wrld.width()):
            for y_coordinate in range(wrld.height()):
                if wrld.exit_at(x_coordinate, y_coordinate):
                    return (x_coordinate, y_coordinate)   
                         
    def get_neighbors(self, wrld, current) -> list[tuple]: # Returns a list of tuples of the surrounding empty nodes. Assumes the exit node is empty
        
        # TODO:FIND MONSTERS TOO
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

    def next_step(self, wrld, path=None) -> None:
        if path:
            current_node = path[0]
            next_node = path[1]

            dx = next_node[0] - current_node[0]
            dy = next_node[1] - current_node[1]
            
            if wrld.empty_at(self.x + dx, self.y + dy):
                self.move(dx, dy)
            else: 
                print(Fore.RED + f"I placed a bomb :D")
                self.place_bomb()
                self.bomb_loc = (self.x, self.y)
                self.bomb_placed_time = self.timestep
        elif self.state == Enum.BOMBING:
            if wrld.empty_at(self.x-1, self.y-1):
                self.move(-1, -1)
            elif wrld.empty_at(self.x+1, self.y-1): 
                self.move(1, -1)
        else:
            print("cry")

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
    
    def trapped_with_monster(self, wrld, path_to_exit, path_to_monster): 
        exit_trapped = False
        monster_trapped = False
        
        for cell in path_to_monster:
            if wrld.wall_at(cell[0], cell[1]):
                monster_trapped = True

        for cell in path_to_exit:
            if wrld.wall_at(cell[0], cell[1]):
                exit_trapped = True
                
        return not monster_trapped and exit_trapped

    def is_by_wall(self, wrld, path): 
        return wrld.wall_at(path[1][0], path[1][1]) 

    def is_in_blast_radius(self, state = None): 
        if not state:
            state = (self.x, self.y)
        if self.bomb_loc:
            return state[0] != self.bomb_loc[0] and abs(state[0] - self.bomb_loc[0]) < 5 and state[1] != self.bomb_loc[1] and abs(state[1] - self.bomb_loc[1]) < 5
        else:
            return False
    
    def feature_calculator(self, wrld): 
        """The sole job of this is to evaluate its setup"""
        # state contains (x,y)
        # monster_loc (int, int) / monster_count (int)
        # bomb_loc (int, int)
        #     if bomb_loc >bomb_timer (in)
        #     if bomb_loc -> in_bomb_pth (bool)
        # exit_loc (int, int)

        ### Getting all information needed for features ###
        # In the end you have
        # average_monster_distance, monster_count, exit_distance, bomb_distance (in_bomb_path), bomb_exists_time
        # communication since the inchwors can communicate with structures, they get information about the other inchworms paths within the strucutre
        if isinstance(wrld, SensedWorld):
            character = wrld.me(self)
            state = (character.x, character.y)
        else: 
            state = (self.x, self.y)

        monsters = self.check_for_monster(wrld, state)
        monster_dists = []
        for monster in monsters:
            monster_loc = (monster.x, monster.y)
            monster_dist = self.heuristic(state, monster_loc)
            monster_dists.append(monster_dist)
        monster_count = len(monsters)

        average_monster_distance = 0
        if monster_count:
            for monster_dist in monster_dists:
                average_monster_distance += monster_dist
            average_monster_distance = average_monster_distance / monster_count
        
            closest_monster_dist = min(monster_dists)

        exit_loc = self.exit
        exit_path = self.plan_path(wrld, state, exit_loc)
        exit_dist = len(exit_path)
        
        # bomb_loc = character.bomb_loc # assuming self.bomb_loc will be None if bomb does not exist
        bomb_dangerzone = False
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            bomb_exists_time = bomb_obj.timer
            bomb_dist = self.heuristic(state, bomb_loc)
            bomb_dangerzone = self.is_in_blast_radius(state) # True/False
        else:
            bomb_dist = None

        """f1=monster_dist, f2=monster_count, f3=exit_dist, f4=bomb_dist, f5=in_bomb_path + bomb_time_existing,"""
        f1 = 1 / (average_monster_distance + 1)     # the larger the average monster distance, the smaller the f
        f2 = monster_count / 2                      # 1.0, 0.5, or 0.0
        f3 = 1 / (exit_dist + 1)                    # longer the distance, the smaller the f
        f4 = 0
        if bomb_dist:
            f4 = 1 / (bomb_dist + 1)                # longer the bomb distance, the smaller the f
        f5 = 0                                      # dont account feature if not in bomb zone
        if bomb_dangerzone:
            f5 = 1 / (bomb_exists_time + 1)        # longer the time goes on, the larger f5 gets
        f6 = 0
        if monster_count:
            f6 = 1/ (closest_monster_dist + 1)
        f7 = 0
        if wrld.explosion_at(character.x, character.y): 
            f7 = 1
        f8 = 0
        if self.is_by_wall(wrld, exit_path): 
            f8 = 1
        return [f1, f2, f3, f4, f5, f6, f7, f8]
        


    def reward_calculator(self, wrld, state):
        x, y = state
        if wrld.empty_at(x, y): 
            reward = 1
        elif wrld.exit_at(x, y): 
            reward = 5000
        elif wrld.wall_at(x, y): 
            reward = -1
        # elif wrld.bomb_at(x, y): 
        #     reward = 0
        elif wrld.monsters_at(x, y):
            reward = -1000
        elif wrld.explosion_at(x, y): 
            reward = -5000
        else:
            reward = 1

        return reward
    
    def next_sensed_wrld(self, wrld, action):
        action_dictionary = { 
            'N': (0, -1), 
            'NW': (-1, -1), 
            'W' : (-1, 0), 
            'SW' : (-1, 1), 
            'S': (0, 1), 
            'SE': (1, 1), 
            'E': (1, 0), 
            'NE': (1, -1), 
            "stay": (0, 0), 
            "bomb" :(0, 0) }
        character = wrld.me(self)
        if action == "bomb":
            character.move(0, 0)
            character.place_bomb() 
            # character.bomb_loc = (character.x, character.y)
            # character.bomb_placed_time = character.timestep
        else: 
            (dx, dy) = action_dictionary[action]
            character.move(dx, dy)
        sensed_world, events = wrld.next()
        return sensed_world
           
    def q_learning(self, sensed_wrld, state):
        alpha = 0.5
        gamma = 0.9
        actions = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE', "stay", "bomb"] # all player moves

        features = self.feature_calculator(sensed_wrld)
        
        # Q(s, a) = w1f1 + w2f2 + ...etc
        q = 0
        for i in range(len(self.weights)):
            q += self.weights[i] * features[i]

        max_Q = -math.inf
        # Max Q(s', a') part
        for a in actions: 
            new_sensed_wrld = self.next_sensed_wrld(sensed_wrld, a)
            Q_next_state = 0
            if new_sensed_wrld.me(self):
                new_features = self.feature_calculator(new_sensed_wrld)
                for i in range(len(self.weights)):
                    Q_next_state += self.weights[i] * new_features[i]
            if Q_next_state > max_Q: 
                max_Q = Q_next_state
            
        r = self.reward_calculator(sensed_wrld, state)
        delta = (r + gamma * max_Q) - q
        
        # Update weights
        for i in range(len(self.weights)):
            self.weights[i] = self.weights[i] + alpha * delta * features[i]
    
        
    def pick_best_action(self, wrld): 
        actions = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE', "stay", "bomb"] # all player moves
        # where wrld is a real wrld or a sensed world 
        max_Q = -math.inf
        # Max Q(s', a') part
        best_action = "bomb"
        for a in actions: 
            new_sensed_wrld = self.next_sensed_wrld(wrld, a)
            Q_next_state = 0
            if new_sensed_wrld.me(self):
                new_features = self.feature_calculator(new_sensed_wrld)
                for i in range(len(self.weights)):
                    Q_next_state += self.weights[i] * new_features[i]
            if Q_next_state > max_Q: 
                max_Q = Q_next_state
                best_action = a
        return best_action

    def training(self, wrld):
        for iteration in range(10): 
            # Make new world for each iteration
            sensed_world = SensedWorld.from_world(wrld)
            finished_game = False
            while not finished_game:
                character = sensed_world.me(self)
                self.q_learning(sensed_world, (character.x, character.y))
                self.pick_best_action(sensed_world)
                sensed_world, events = sensed_world.next()

                for event in events:
                    if event.tpe == 2 or event.tpe == 3 or event.tpe == 4:
                        finished_game = True

            print(f"Weights are {self.weights} for iteration {iteration}")
                  
    def do(self, wrld):
        # Your code here
        self.exit = self.locate_exit(wrld)
        start = (self.x, self.y)
        monster_loc = None
        by_wall = False
        trapped_with_monster = False
        path_to_monster = []

        if self.check_for_monster(wrld, (self.x, self.y)):
            first_seen_monster = self.check_for_monster(wrld, (self.x, self.y))[0]
            monster_loc = (first_seen_monster.x, first_seen_monster.y)
            print("update path to monster")
            path_to_monster = self.plan_path(wrld, start, monster_loc)
        
        print("path to monster: ", path_to_monster)

        path = self.plan_path(wrld, start, self.exit)
        if path:
            by_wall = self.is_by_wall(wrld, path)
        if path_to_monster: 
            trapped_with_monster = self.trapped_with_monster(wrld, path_to_monster)
            print("trapped with monster: ", trapped_with_monster)

        not_in_range_of_bomb = self.is_in_blast_radius()
        time_left = self.timestep - self.bomb_placed_time
        if time_left > 10: 
            self.bomb_loc = None
        # TODO: use time left in bomb to determine actions

        print(f"state: {self.state}")

        match self.state:
            case Enum.TRAVELING:
                if monster_loc and trapped_with_monster: 
                    self.training(wrld)
                    self.state = Enum.FLEEING
                elif by_wall:
                    # Move to wall, then place Bomb
                    self.color_path(path)
                    self.next_step(wrld, path)
                    # Switch States
                    self.state = Enum.BOMBING
                else: 
                    # A* walking
                    self.color_path(path)
                    self.next_step(wrld, path)
            case Enum.BOMBING:
                if not_in_range_of_bomb: # Player in danger of explosion
                    self.state = Enum.WAITING
                else: 
                    self.next_step(wrld)
            case Enum.FLEEING:
                if not monster_loc: 
                    if self.bomb_loc: # Bomb is on map
                        self.state = Enum.BOMBING
                    else:
                        self.color_path(path)
                        self.next_step(wrld, path)
                        self.state = Enum.TRAVELING
                else:
                    self.pick_best_action(wrld)
                    self.avoid_monster(wrld)               
            case Enum.WAITING: 
                if not self.bomb_loc: 
                    # no bomb
                    self.state = Enum.TRAVELING
                elif monster_loc and trapped_with_monster: 
                    # MONSTER AHH
                    # self.avoid_monster(wrld)
                    self.training(wrld)
                    self.state = Enum.FLEEING
        
        self.timestep += 1