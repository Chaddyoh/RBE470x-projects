# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from colorama import Fore, Back
import heapq
import math
import random

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
    monsters = []
    # weights = [1, -1, 1, -1, -1, -1, -1, -1, 1, 1,1]
    weights = [-2.354353462537136, -0.5323870432714803, -0.051915622304485744, -0.9487924376891508, 0.08239716923971455, -0.16674062808966658, -0.11437727626035486, -0.9759391605635761, -0.26977066419196155, -0.3749298013886435]
        
    def locate_exit(self, wrld) -> tuple: # Returns X,Y tuple for exit
        for x_coordinate in range(wrld.width()):
            for y_coordinate in range(wrld.height()):
                if wrld.exit_at(x_coordinate, y_coordinate):
                    return (x_coordinate, y_coordinate)   

    def get_walkables(self, wrld, current) -> list[tuple]:
        neighbors = []
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                if (current[0]+dx >=0) and (current[0]+dx < wrld.width()) and (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                    if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy) or wrld.bomb_at(current[0]+dx, current[1]+dy):
                        neighbors.append((current[0]+dx, current[1]+dy))
        return neighbors

    def get_walkable_actions(self, wrld, current, training=False): 
        walkable_actions = []
        direction_to_action = {
            (-1, -1) : "NW",
            (-1, 1) : "SW",
            (-1, 0) : "W",
            (1, -1) : "NE", 
            (1, 1) : "SE", 
            (1, 0) : "E",
            (0, -1) : "N",
            (0, 1) : "S",
            (0, 0) : "stay"
        }
        if not wrld.bombs:
            walkable_actions.append("bomb")
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                state = (current[0]+dx, current[1]+dy)
                if (current[0]+dx >=0) and (current[0]+dx < wrld.width()) and (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                    # Check if the cell is walkable
                    technically_walkable = wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy) or wrld.bomb_at(current[0]+dx, current[1]+dy)

                    # Check Bomb is about to explode / explosions on floor
                    bomb_exists_time = 10
                    bomb_dangerzone = False
                    if wrld.bombs: 
                        bomb_obj = list(wrld.bombs.values())[0]
                        bomb_loc = (bomb_obj.x, bomb_obj.y)
                        bomb_exists_time = bomb_obj.timer
                        bomb_dangerzone = self.is_in_blast_radius(wrld, state) 

                    # Check thats at least one away from monster
                    monster_dangerzone = False
                    if wrld.monsters:
                        monster = list(wrld.monsters.values())[0][0]
                        monster_dangerzone = self.monster_range(wrld, state, monster)
                    
                    if training:
                        if technically_walkable:
                            walkable_actions.append(direction_to_action[(dx, dy)])
                    else:
                        if technically_walkable and (not bomb_dangerzone or (bomb_exists_time > 2)) and not monster_dangerzone:
                            walkable_actions.append(direction_to_action[(dx, dy)])
                            
        if not training:
            print("i've finished training, and here are my walkable actions: ", walkable_actions)
        return walkable_actions
        

    def get_neighbors(self, wrld, current) -> list[tuple]: # Returns a list of tuples of the surrounding nodes. Assumes the exit node is empty
        
        # TODO:FIND MONSTERS TOO
        neighbors = []
        for dx in [-1,0,1]:
            if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
                for dy in [-1,0,1]:
                    if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                        if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy) or wrld.wall_at(current[0]+dx, current[1]+dy) or wrld.monsters_at(current[0]+dx, current[1]+dy):
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
        elif wrld.monsters_at(location[0], location[1]):
            return 6
    
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
        if self.state == Enum.BOMBING or self.state == Enum.WAITING:
            print("x:", self.x, "\ty:", self.y)
            walkable_neighbors = self.get_walkables(wrld, (self.x, self.y))
            safe_cells = []
            for cell in walkable_neighbors:
                if not self.is_in_blast_radius(wrld, cell):
                    safe_cells.append(cell)
            if len(safe_cells) > 0:
                chosen_cell = random.choice(safe_cells)    
                dx = chosen_cell[0]-self.x 
                dy = chosen_cell[1]-self.y
            else: 
                dx, dy = 0, 0
            print(f"moving to avoid the bomb in {dx}, {dy}")
            self.move(dx, dy)
            
        elif path:
            current_node = path[0]
            next_node = path[1]

            dx = next_node[0] - current_node[0]
            dy = next_node[1] - current_node[1]
            
            if wrld.empty_at(self.x + dx, self.y + dy) or wrld.exit_at(self.x + dx, self.y + dy):
                self.move(dx, dy)
            else: 
                print(Fore.RED + f"I placed a bomb :D")
                self.move(0,0)
                self.place_bomb()
        else:
            print("cry")

    def check_for_monster(self, wrld, current) -> tuple: 
        global monsters
        monsters_list = []
        for dx in [-4,-3,-2,-1,0,1,2,3, 4]:
            if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
                for dy in [-4,-3,-2,-1,0,1,2,3,4]:
                    if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                        monster_square = wrld.monsters_at(current[0]+dx, current[1]+dy)
                        if monster_square:
                            monsters_list += monster_square
        return monsters_list 

    def monster_range(self, wrld, state, monster):
        for dx in [-2,-1,0,1,2]:
            search_x = state[0] + dx
            if (search_x>=0) and (search_x<wrld.width()):
                for dy in [-2,-1,0,1,2]:
                    search_y = state[1] + dy
                    if (search_y>=0) and (search_y<wrld.width()):
                        if search_x == monster.x and search_y == monster.y:
                            return True
        return False

    def is_valid_space(self, wrld, loc):
        if loc[0] > 0 and loc[0] < wrld.width() and loc[1] > 0 and loc[1] < wrld.height(): # if state location is within the map 
            return wrld.empty_at(loc[0], loc[1]) # Code to check if empty except for wall: wrld.exit_at(loc[0], loc[1]) or wrld.bomb_at(loc[0], loc[1]) or wrld.explosion_at(loc[0], loc[1]) or wrld.monsters_at(loc[0], loc[1]) or wrld.characters_at(loc[0], loc[1]) 
               
        return False
    
    def trapped_with_monster(self, wrld, path_to_exit, path_to_monster): 
        # exit_trapped = False
        monster_trapped = False
        
        for cell in path_to_monster:
            if wrld.wall_at(cell[0], cell[1]):
                monster_trapped = True

        # for cell in path_to_exit:
        #     if wrld.wall_at(cell[0], cell[1]):
        #         exit_trapped = True
                
        return not monster_trapped

    def is_by_wall(self, wrld, path): 
        return wrld.wall_at(path[1][0], path[1][1]) or wrld.wall_at(path[2][0], path[2][1]) 

    def is_in_blast_radius(self, wrld, state = None): 
        if not state:
            state = (self.x, self.y)
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            return (state == bomb_loc) or (state[0] == bomb_loc[0] and abs(state[0] - bomb_loc[0]) < 5) or (state[1] == bomb_loc[1] and abs(state[1] - bomb_loc[1]) < 5)
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
        closest_monster = None
        closest_monster_dist = math.inf
        for monster in monsters:
            monster_loc = (monster.x, monster.y)
            monster_dist = self.heuristic(state, monster_loc)
            if monster_dist < closest_monster_dist:
                closest_monster_dist = monster_dist
                closest_monster = monster
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
        if exit_path:
            exit_euc_dist = self.heuristic(state, exit_path[1])
        
        # bomb_loc = character.bomb_loc # assuming self.bomb_loc will be None if bomb does not exist
        bomb_dangerzone = False
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            bomb_exists_time = bomb_obj.timer
            bomb_dist = self.heuristic(state, bomb_loc)
            bomb_dangerzone = self.is_in_blast_radius(wrld, state) # True/False
        else:
            bomb_dist = None
        
        if monster_count and exit_path:
            monster_exit_dist = self.heuristic((closest_monster.x, closest_monster.y), exit_path[1])
            try:
                angle = math.acos(((monster_exit_dist**2 - exit_euc_dist**2 - closest_monster_dist**2) / (-2 * exit_euc_dist * closest_monster_dist)))
            except:
                angle = 0
        
        """f1=monster_dist, f2=monster_count, f3=exit_dist, f4=bomb_dist, f5=in_bomb_path + bomb_time_existing,"""
        f1 = average_monster_distance**2            # the larger the average monster distance, the smaller the f
        
        f2 = monster_count / 2                      # 1.0, 0.5, or 0.0
        f3 = 1 / (exit_dist + 1)                    # longer the distance, the smaller the f
        f4 = 0
        if bomb_dist:
            f4 = bomb_dist                          # longer the bomb distance, the bigger the f
        f5 = 0                                      # dont account feature if not in bomb zone
        if bomb_dangerzone:
            f5 = 1 / (bomb_exists_time + 1)         # longer the time goes on, the larger f5 gets
        f6 = 0
        if monster_count:
            f6 = 1/ (closest_monster_dist + 1)
        f7 = 1/ (1 + len(self.get_walkables(wrld, (character.x, character.y)))) #if in corner

        f8 = 0
        if monster_count and exit_path:
            if angle > 0: # f9 = 1 if there is an angle between monster and next part of path
                f8 = 1
        f9 = 0
        f10 = 0
        if monster_count:
            f9 = 1/(abs(state[0] - monster_loc[0])+1) # Horizantal dist to monster
            f10 = 1/(abs(state[1] - monster_loc[1])+1) # Vertical distance to monster
        features = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10 ]

        # Normalize features
        for i in range(len(features)):
            features[i] = features[i]/sum(features) 

        return features


    def reward_calculator(self, wrld, state):
        x, y = state
        try:
            if wrld.empty_at(x, y): 
                reward = -1
            elif wrld.exit_at(x, y): 
                print("q learning found the exit")
                reward = 5000
            elif wrld.wall_at(x, y): 
                reward = -50
            # elif wrld.bomb_at(x, y): 
            #     reward = 0
            elif wrld.monsters_at(x, y):
                reward = -5000
            elif wrld.explosion_at(x, y): 
                reward = -5000
            else:
                reward = -1
        except:
            reward = -10

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
        (dx, dy) = action_dictionary[action]

        if action == "bomb":
            character.place_bomb() 
            # print(Fore.RED + f"Character placed a bomb :D")
        character.move(dx, dy)
        sensed_world, events = wrld.next()

        return sensed_world, (dx, dy)
    
    def action_based_movement(self, action):
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
        (dx, dy) = action_dictionary[action]

        if action == "bomb":
            self.place_bomb() 
            print(Fore.RED + f"Action based movement: I placed a bomb :D")
        print("Movement time ! ", action)
        self.move(dx, dy)

    def q_learning(self, sensed_wrld, state):
        alpha = 0.5
        gamma = 0.9
        actions = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE', "stay", "bomb"] # all player moves

        features = self.feature_calculator(sensed_wrld)
        
        # Calculate the Q value of this state first 
        # Q(s, a) = w1f1 + w2f2 + ...etc
        q = 0
        for i in range(len(self.weights)):
            q += self.weights[i] * features[i]

        max_Q = -math.inf
        r = self.reward_calculator(sensed_wrld, state)
        # Max Q(s', a') part (for all next states)
        for a in actions: 
            new_sensed_wrld, (dx, dy) = self.next_sensed_wrld(sensed_wrld, a)
            Q_next_state = 0
            if not new_sensed_wrld.me(self): # if character dies
                prev_character = list(sensed_wrld.characters.values())[0][0]
                prev_character.x += dx
                prev_character.y += dy
                # add movement for character (x,y)
                new_sensed_wrld.characters[78] = [prev_character]
            new_features = self.feature_calculator(new_sensed_wrld)
            for i in range(len(self.weights)):
                    Q_next_state += self.weights[i] * new_features[i]
            if Q_next_state > max_Q: 
                max_Q = Q_next_state
                r = self.reward_calculator(new_sensed_wrld, (state[0]+dx, state[1]+dy))
            
        delta = (r + gamma * max_Q) - q
        
        # Update weights
        for i in range(len(self.weights)):
            self.weights[i] = self.weights[i] + alpha * delta * features[i]
        
    
        
    def pick_best_action(self, wrld, state, training=False): 
        # actions = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE', "stay", "bomb"] # all player moves
        actions = self.get_walkable_actions(wrld, state, training)
        # where wrld is a real wrld or a sensed world 
        max_Q = -math.inf
        # Max Q(s', a') part
        best_action = "stay"
        for a in actions:
            new_sensed_wrld, (dx, dy) = self.next_sensed_wrld(wrld, a)
            Q_next_state = 0
            if not new_sensed_wrld.me(self):
                prev_character = list(wrld.characters.values())[0][0]
                prev_character.x += dx
                prev_character.y += dy
                # add movement for character (x,y)
                new_sensed_wrld.characters[78] = [prev_character]
                
            
            new_features = self.feature_calculator(new_sensed_wrld)
            
            for i in range(len(self.weights)):
                    Q_next_state += self.weights[i] * new_features[i]
            if Q_next_state > max_Q: 
                max_Q = Q_next_state
                best_action = a
        return best_action

    def training(self, wrld, iter_count):
        # Normalize weights
        dividend = sum(map(abs, self.weights)) #(max(self.weights) - min(self.weights))
        for i in range(len(self.weights)):
            self.weights[i] = self.weights[i]/dividend

        for iteration in range(iter_count): 
            # Make new world for each iteration
            sensed_world = SensedWorld.from_world(wrld)
            finished_game = False
            step = 0
            while (not finished_game) and step < 25:
                character = sensed_world.me(self)
                self.q_learning(sensed_world, (character.x, character.y))
                # actions = self.get_walkable_actions(wrld, (character.x, character.y))
                # action = random.choice(actions)
                action = self.pick_best_action(sensed_world, (character.x, character.y), True)
                sensed_world, (dx, dy) = self.next_sensed_wrld(sensed_world, action)
                # sensed_world, events = sensed_world.next()

                for event in sensed_world.events:
                    if event.tpe == 2 or event.tpe == 3 or event.tpe == 4:
                        finished_game = True
                step += 1

            print(f"Weights are {self.weights} for iteration {iteration}")
                  
    def do(self, wrld):
        # Your code here
        self.exit = self.locate_exit(wrld)
        start = (self.x, self.y)
        monster_loc = None
        by_wall = False
        trapped_with_monster = False
        path_to_monster = []
        bomb_exists = bool(wrld.bombs)
        
        if self.check_for_monster(wrld, (self.x, self.y)):
            first_seen_monster = self.check_for_monster(wrld, (self.x, self.y))[0]
            monster_loc = (first_seen_monster.x, first_seen_monster.y)
            # print("update path to monster")
            path_to_monster = self.plan_path(wrld, start, monster_loc)
        
        # print("path to monster: ", path_to_monster)

        path = self.plan_path(wrld, start, self.exit)
        if len(path) > 2:
            by_wall = self.is_by_wall(wrld, path)
        if path_to_monster: 
            trapped_with_monster = self.trapped_with_monster(wrld, path, path_to_monster)
            print("trapped with monster: ", trapped_with_monster)

        in_range_of_bomb = self.is_in_blast_radius(wrld)
        # TODO: use time left in bomb to determine actions

        print(f"state: {self.state}")

        match self.state:
            case Enum.TRAVELING:
                if monster_loc and trapped_with_monster: 
                    self.training(wrld, 10)
                    action = self.pick_best_action(wrld, (self.x, self.y))
                    self.action_based_movement(action)
                    self.state = Enum.FLEEING
                elif by_wall:
                    if bomb_exists:
                        self.state = Enum.WAITING
                        self.next_step(wrld, path)
                    else:
                        # Move to wall, then place Bomb
                        self.color_path(path)
                        self.next_step(wrld, path)
                        if wrld.bombs:
                            # Switch States
                            self.state = Enum.BOMBING
                else: 
                    # A* walking
                    self.color_path(path)
                    self.next_step(wrld, path)
            case Enum.BOMBING:
                if monster_loc and trapped_with_monster:
                    self.training(wrld, 10)
                    action = self.pick_best_action(wrld, (self.x, self.y))
                    self.action_based_movement(action)
                    self.state = Enum.FLEEING
                elif not in_range_of_bomb: # Player in danger of explosion
                    self.state = Enum.WAITING
                else: 
                    self.next_step(wrld)
            case Enum.FLEEING:
                if not monster_loc: 
                    if bomb_exists: # Bomb is on map
                        self.color_path(path)
                        self.state = Enum.WAITING
                        self.next_step(wrld, path)
                    else:
                        self.color_path(path)
                        self.next_step(wrld, path)
                        self.state = Enum.TRAVELING
                else:
                    action = self.pick_best_action(wrld, (self.x, self.y))
                    self.action_based_movement(action)
                    # self.avoid_monster(wrld)               
            case Enum.WAITING: 
                if monster_loc and trapped_with_monster: 
                    # MONSTER AHH
                    # self.avoid_monster(wrld)
                    self.training(wrld, 10)
                    action = self.pick_best_action(wrld, (self.x, self.y))
                    self.action_based_movement(action)
                    self.state = Enum.FLEEING
                elif not bomb_exists: 
                    # no bomb
                    self.move(0,0)
                    print("i should be staying still before traveling")
                    self.state = Enum.TRAVELING
                elif in_range_of_bomb:
                    self.next_step(wrld, path)
                else:
                    self.move(0,0)
                    print("i should be staying still")
                
        
