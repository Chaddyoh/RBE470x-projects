# This is necessary to find the main code

import sys
sys.path.insert(0, '../../bomberman')
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
    timestep = 0
    monsters = []
    weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    bomb_loc = None
    bomb_placed_time = 0
    is_training = True
    epsilon = 0.1
    ddx = 0
    ddy = 0
    dwallcount =24

    def locate_exit(self, wrld) -> tuple: # Returns X,Y tuple for exit
        for x_coordinate in range(wrld.width()):
            for y_coordinate in range(wrld.height()):
                if wrld.exit_at(x_coordinate, y_coordinate):
                    return (x_coordinate, y_coordinate)

    # def get_walkables(self, wrld, current) -> list[tuple]:
    #     neighbors = []
    #     for dx in [-1,0,1]:
    #         for dy in [-1,0,1]:
    #             if (current[0]+dx >=0) and (current[0]+dx < wrld.width()) and (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
    #                 if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy) or wrld.bomb_at(current[0]+dx, current[1]+dy):
    #                     neighbors.append((current[0]+dx, current[1]+dy))
    #     return neighbors


    # def get_neighbors(self, wrld, current) -> list[tuple]: # Returns a list of tuples of the surrounding empty nodes. Assumes the exit node is empty

    #     # TODO:FIND MONSTERS TOO
    #     neighbors = []
    #     for dx in [-1,0,1]:
    #         if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
    #             for dy in [-1,0,1]:
    #                 if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
    #                     if wrld.empty_at(current[0]+dx, current[1]+dy) or wrld.exit_at(current[0]+dx, current[1]+dy) or wrld.wall_at(current[0]+dx, current[1]+dy) or wrld.monsters_at(current[0]+dx, current[1]+dy):
    #                         neighbors.append((current[0]+dx, current[1]+dy))
    #     return neighbors

    # def heuristic(self, point1, point2) -> float:
    #     euclidean_dist = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    #     return euclidean_dist

    # def cost(self, wrld, location):
    #     if wrld.empty_at(location[0], location[1]) or wrld.exit_at(location[0], location[1]):
    #         return 1
    #     elif wrld.wall_at(location[0], location[1]):
    #         return 13
    #     elif wrld.monsters_at(location[0], location[1]):
    #         return 6

    # def plan_path(self, wrld, start, goal) -> list[tuple]:
    #     frontier = PriorityQueue()
    #     frontier.put(start, 0)
    #     came_from = {}
    #     cost_so_far = {}
    #     came_from[start] = None
    #     cost_so_far[start] = 0

    #     while not frontier.empty():
    #         current = frontier.get()

    #         for next in self.get_neighbors(wrld, current):
    #             self.set_cell_color(next[0], next[1], Fore.WHITE + Back.MAGENTA)
    #             new_cost = cost_so_far[current] + self.cost(wrld, next)
    #             if next not in cost_so_far or new_cost < cost_so_far[next]:
    #                 cost_so_far[next] = new_cost
    #                 priority = new_cost + self.heuristic(goal, next)
    #                 frontier.put(next, priority)
    #                 came_from[next] = current

    #         if current == goal:
    #             break

    #     try:
    #         path = [goal]
    #         previous_node = goal
    #         while not previous_node == start:
    #             next_node = came_from[previous_node]
    #             path.append(next_node)
    #             previous_node = next_node

    #         path.reverse()
    #     except:
    #         path = []

    #     return path

    # def color_path(self, path) -> None:
    #     for coord in path:
    #         self.set_cell_color(coord[0], coord[1], Fore.BLUE + Back.YELLOW)

    # def next_step(self, wrld, path=None) -> None:
    #     if path:
    #         current_node = path[0]
    #         next_node = path[1]

    #         dx = next_node[0] - current_node[0]
    #         dy = next_node[1] - current_node[1]

    #         if wrld.empty_at(self.x + dx, self.y + dy):
    #             self.move(dx, dy)
    #         else:
    #             #print(Fore.RED + f"I placed a bomb :D")
    #             self.place_bomb()
    #             self.bomb_placed_time = self.timestep
    #     elif self.state == Enum.BOMBING:
    #         #print("x:", self.x, "\ty:", self.y)
    #         if wrld.empty_at(self.x-1, self.y-1):
    #             self.move(-1, -1)
    #         elif wrld.empty_at(self.x+1, self.y-1):
    #             self.move(1, -1)
    #         elif wrld.empty_at(self.x-1, self.y+1):
    #             self.move(-1, 1)
    #         elif wrld.empty_at(self.x+1, self.y+1):
    #             self.move(1, 1)
    #         elif wrld.empty_at(self.x-1, self.y):
    #             self.move(-1, 0)
    #         elif wrld.empty_at(self.x+1, self.y):
    #             self.move(1, 0)
    #         elif wrld.empty_at(self.x-1, self.y):
    #             self.move(0, -1)
    #         elif wrld.empty_at(self.x+1, self.y):
    #             self.move(0, 1)
    #     else:
    #         pass
    #     #print("cry")

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

    # def monster_range(self, wrld, state, monster):
    #     if monster.name == "selfpreserving":
    #         for dx in [-1,0,1]:
    #             search_x = monster.x + dx
    #             if (search_x>=0) and (search_x<wrld.width()):
    #                 for dy in [-1,0,1]:
    #                     search_y = monster.y + dy
    #                     if (search_y>=0) and (search_y<wrld.width()):
    #                         if state == (search_x, search_y):
    #                             return True
    #     elif monster.name == "aggressive":
    #         for dx in [-2,-1,0,1,2]:
    #             search_x = monster.x + dx
    #             if (search_x>=0) and (search_x<wrld.width()):
    #                 for dy in [-2,-1,0,1,2]:
    #                     search_y = monster.y + dy
    #                     if (search_y>=0) and (search_y<wrld.width()):
    #                         if state == (search_x, search_y):
    #                             return True
    #     return False

    # def is_valid_space(self, wrld, loc):
    #     if loc[0] > 0 and loc[0] < wrld.width() and loc[1] > 0 and loc[1] < wrld.height(): # if state location is within the map
    #         return wrld.empty_at(loc[0], loc[1]) # Code to check if empty except for wall: wrld.exit_at(loc[0], loc[1]) or wrld.bomb_at(loc[0], loc[1]) or wrld.explosion_at(loc[0], loc[1]) or wrld.monsters_at(loc[0], loc[1]) or wrld.characters_at(loc[0], loc[1])
    #     return False

    # def trapped_with_monster(self, wrld, path_to_exit, path_to_monster):
    #     exit_trapped = False
    #     monster_trapped = False

    #     for cell in path_to_monster:
    #         if wrld.wall_at(cell[0], cell[1]):
    #             monster_trapped = True

    #     for cell in path_to_exit:
    #         if wrld.wall_at(cell[0], cell[1]):
    #             exit_trapped = True

    #     return not monster_trapped and exit_trapped

    # def is_by_wall(self, wrld, path):
    #     return wrld.wall_at(path[1][0], path[1][1])

    # def is_in_blast_radius(self, wrld, state = None):
    #     if not state:
    #         state = (self.x, self.y)
    #     if wrld.bombs:
    #         bomb_obj = list(wrld.bombs.values())[0]
    #         bomb_loc = (bomb_obj.x, bomb_obj.y)
    #         return state[0] == bomb_loc[0] and abs(state[0] - bomb_loc[0]) < 5 and state[1] == bomb_loc[1] and abs(state[1] - bomb_loc[1]) < 5
    #     else:
    #         return False

#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************
#*******************************************************************************

    def count_walls(self, wrld):
        count = 0
        for y in [3, 7, 11]:
            for x in range(wrld.width()):
                if wrld.wall_at(x,y):
                    count+=1
        return count

    def pick_best_action(self, wrld, state):
        print("PICK BEST ACTION BEGIN ------------------------------------------------")
        actions = self.get_walkable_actions(wrld, state)
        print("WALKABLE ACTIONS: ", actions)
        max_Q = -math.inf
        # Max Q(s', a') part
        q_values = {}
        for a in actions:
            features = self.feature_calculator(wrld, a)  # Features depend on action
            q_values[a] = sum(self.weights[i] * features[i] for i in range(len(self.weights)))
            print("CURRENT ACTION FEATURES: ", features)      
        print("All q-values for current: ", q_values)
        max_action = max(q_values, key=q_values.get)
        q = q_values[max_action]

        return max_action

    def get_walkable_actions(self, wrld, state):
        walkable_actions = []
        #print("WALL AT TEST: ", wrld.wall_at(0,3))
        #print(wrld)
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                #print("STATE: ", state," ACTION: ", self.action_to_delta((dx, dy)), " DXDY: ", (dx, dy), " POS:", state[0]+dx, state[1]+dy)
                if (state[0]+dx >=0) and (state[1]+dy >=0) :
                    if  (state[0]+dx < wrld.width()) and (state[1]+dy < wrld.height()):
                        if wrld.empty_at(state[0]+dx, state[1]+dy) or wrld.exit_at(state[0]+dx, state[1]+dy) or (dx, dy) == (0,0):
                            if not wrld.bomb_at(state[0]+dx, state[1]+dy) or not wrld.explosion_at(state[0]+dx, state[1]+dy):
                                if not wrld.wall_at(state[0]+dx, state[1]+dy):
                                        walkable_actions.append(self.action_to_delta((dx, dy)))
        if not wrld.bombs:
            walkable_actions.append("bomb")
        return walkable_actions

    def feature_calculator(self, wrld, a):
        """
        Calculate features for a given state-action pair in a Q-learning agent.
        
        Args:
            wrld: The game world (assumed to be a SensedWorld object).
            a: The action to evaluate (e.g., 'up', 'down', 'stay', 'bomb').
        
        Returns:
            list: A list of 10 normalized features.
        """
        # Get the character's current position
        character = wrld.me(self)
        current_state = (character.x, character.y)
        
        # Compute the next state based on the action
        delta = self.action_to_delta(a)
        if isinstance(wrld, SensedWorld):
            character = wrld.me(self)
            #print("FEATURE CALULATOR STAT: ", character.x, character.y)
            if type(delta) == tuple:
                next_state = (character.x+delta[0], character.y+delta[1])
            else:
                next_state = (character.x, character.y)
        else:
            if type(delta)==tuple:
                next_state = (self.x+delta[0], self.y+delta[1])
            else:
                next_state = (self.x, self.y)
        
        # Feature 1: Normalized Manhattan distance to exit
        exit_loc = self.exit  # Assumes self.exit is set to (exit_x, exit_y)
        manhattan_dist = abs(exit_loc[0] - next_state[0]) + abs(exit_loc[1] - next_state[1])
        max_manhattan = wrld.width() + wrld.height()  # Maximum possible distance in grid
        f1 = manhattan_dist / max_manhattan
        
        # Feature 2: Inverse distance to closest monster
        monsters = self.check_for_monster(wrld, current_state)  # Returns list of monster objects
        if wrld.monsters:
            monster_obj = list(wrld.monsters.values())[0]  # Consider the first bomb (simplification)
            print(wrld.monster)
            monster_loc = (monster_obj.x, monster_obj.y)
            monster_dist = abs(monster_loc[0] - next_state[0]) + abs(monster_loc[1] - next_state[1])
            f2 = monster_dist / max_manhattan  # Normalized distance to bomb

        else:
            f2 = 0
        
        # Features 3-5: Bomb-related features
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]  # Consider the first bomb (simplification)
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            bomb_timer = bomb_obj.timer
            bomb_dist = abs(bomb_loc[0] - next_state[0]) + abs(bomb_loc[1] - next_state[1])
            f3 = bomb_dist / max_manhattan  # Normalized distance to bomb
            
            # Check if next state is in blast radius (assuming radius of 3 tiles)
            blast_radius = 4
            in_blast_radius = (abs(bomb_loc[0] - next_state[0]) <= blast_radius and bomb_loc[1] == next_state[1]) or \
                            (abs(bomb_loc[1] - next_state[1]) <= blast_radius and bomb_loc[0] == next_state[0])
            f4 = 1 if in_blast_radius else 0  # Binary feature
            
            f5 = 1 / (bomb_timer + 1)  # Inverse timer, 0 < f5 <= 1
        else:
            f3 = 0  # No bomb, distance is irrelevant
            f4 = 0  # Not in blast radius
            f5 = 0  # No timer
        
        # Features 6-7: Action-specific indicators
        f6 = 1 if a == 'stay' else 0  # Is the action 'stay'?
        f7 = 1 if a == 'bomb' else 0  # Is the action 'bomb'?
        
        # Feature 8: Normalized number of walkable actions
        walkable_actions = self.get_walkable_actions(wrld, current_state)  # List of valid actions
        f8 = len(walkable_actions) / 4.0  # Normalize by max 4 directions
        
        # Feature 9: Explosion at next state
        f9 = 1 if wrld.explosion_at(next_state[0], next_state[1]) else 0
        
        # Feature 10: Bias term
        f10 = 1  # Constant feature for Q-function offset
        
        # Return the feature vector
        features = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10]
        return features

    def reward_calculator(self, wrld, state):
        x, y = state
        #print("CALCULATING REWARD: ", state)
        #print("WORLD GRID: ", wrld.width(), wrld.height())
        reward = 0
        exit_loc = self.locate_exit(wrld)
        if (exit_loc[0]-x) > (exit_loc[0] - self.ddx):
            #print("REWARD Getting Closer in the X")
            reward += 10
        else:
            reward +=-2
        if (exit_loc[1]-y) > (exit_loc[1] - self.ddy):
            #print("REWARD Getting Closer in the Y")
            reward += 30
        else:
            reward +=-2
        if (self.count_walls(wrld)<self.dwallcount):
            #print("REWARD Getting Closer in the Y")
            reward += 1000
        if (x == self.ddx and y ==self.ddy):
           # print("REWARD Staying")
            reward += -90        
        if wrld.exit_at(x, y):
            reward+= 5000
            #print("REWARD FOR EXIT")
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            reward+= 60
            #print("REWARD FOR BOMB")
            if bomb_loc[0] == state[0] or bomb_loc[1] == state[1]:
                reward+= -50
            #    print("REWARD FOR BOMB IN CARDINAL")
        if wrld.monsters_at(x, y):
            reward-= 10 
            #print("REWARD FOR MONSTER AT")
        if wrld.explosion_at(x, y):
            reward+=-30
        reward+=100**(state[1]/exit_loc[1])
            #print("SYM DIED TO EXPLOSION")
        #print("REWARD: ",reward)
        self.ddx = state[0]
        self.ddy = state[1]
        self.dwallcount = self.count_walls(wrld)
        return reward

    def next_sensed_wrld(self, wrld, a):
        character = wrld.me(self)
        #print("STATE: ",character.x, character.y, "ACTION:", action)
        if a == "bomb":
            character.place_bomb()
            dx = 0
            dy = 0
        else:
            (dx, dy) = self.action_to_delta(a)
            character.move(dx, dy)
        sensed_world, events = wrld.next()
        #print("EVENTS FROM NEXT SENSED WORLD: ", events)
        return sensed_world, (dx, dy)

    def action_based_movement(self, a):
        a = self.action_to_delta(a)
        if a == "bomb":
            self.place_bomb()
            self.move(0, 0)
        else:
            (dx, dy) = a
            self.move(dx, dy)

    def action_to_delta(self, a):
        action_dictionary = {
            'N': (0, -1),
            'NW': (-1, -1),
            'W' : (-1, 0),
            'SW' : (-1, 1),
            'S': (0, 1),
            'SE': (1, 1),
            'E': (1, 0),
            'NE': (1, -1),
            'bomb':'bomb',
            'stay':(0,0)
        }
        if type(a) == str:
            return action_dictionary[a]
        if type(a) == tuple:
            for key, val in action_dictionary.items():
                if val == a:
                    return key
            return None


    def q_learning(self, sensed_wrld, state):
        alpha = 0.5
        gamma = 0.9
        actions = self.get_walkable_actions(sensed_wrld, state)
        q_values = {}
        for a in actions:
            features = self.feature_calculator(sensed_wrld, a)  # Features depend on action
            q_values[a] = sum(self.weights[i] * features[i] for i in range(len(self.weights)))
    
        if random.random() < self.epsilon:
            max_action = random.choice(actions)
            #print("RANDOM ACTION CHOSEN")
        else:
            max_action = max(q_values, key=q_values.get)

        q = q_values[max_action]
        features = self.feature_calculator(sensed_wrld, max_action)

        # Simulate the action
        new_sensed_wrld, (dx, dy) = self.next_sensed_wrld(sensed_wrld, max_action)
        next_state = (state[0] + dx, state[1] + dy)
        r = self.reward_calculator(new_sensed_wrld, next_state)
        #print("REWARD FOR NEXT STATE: ",r )

        #print("Compute the max q-val in the next state: ") 

        # Compute max Q-value in next state
        if new_sensed_wrld.me(self):
            next_actions = self.get_walkable_actions(new_sensed_wrld, next_state)
            #print("Next Walkable Actions:", next_actions)
            max_Q_next = -math.inf
            for next_a in next_actions:
                delta = self.action_to_delta(next_a)
                next_features = self.feature_calculator(new_sensed_wrld, delta)
                #print(f"Features of next action {next_a}: {next_features}")
                Q_next = sum(self.weights[i] * next_features[i] for i in range(len(self.weights)))
                max_Q_next = max(max_Q_next, Q_next)
                #print(f"Q({next_a}): {Q_next}")
        else:
            max_Q_next = 0  # Terminal state
        
        # Compute TD error
        delta = r + gamma * max_Q_next - q
        
        # Update weights
        for i in range(len(self.weights)):
            self.weights[i] += alpha * delta * features[i]
        #print("WEIGTHS: ", self.weights)
        #print(f"Max Q: {max_Q_next}, Max action: {max_action}")
        return max_action    
    
    def training(self, wrld, iterations):
        for iteration in range(iterations):
            # Make new world for each iteration
            #self.epsilon = self.epsilon**(iteration/(iterations))
            sensed_world = SensedWorld.from_world(wrld) #  Current State
            character = sensed_world.me(self)
            while character:
                state = (character.x, character.y)
                print(f"STATE: ({character.x:>3}, {character.y:>3})")  # Left-justifies "REWARD" with 16 spaces
                next_action = self.q_learning(sensed_world, state) # Run q-learning which will update weights
                sensed_world, (dx, dy) = self.next_sensed_wrld(sensed_world, next_action)
                character = sensed_world.me(self)

            #print(f"Weights are {self.weights} for iteration {iteration}")
        return False

    def do(self, wrld):
        # Your code here
        self.exit = self.locate_exit(wrld)
        self.epsilon = 0.45
        if self.is_training:
            self.is_training = self.training(wrld, 10000)

        action = self.pick_best_action(wrld, (self.x, self.y))
        #print()
        #print("ACTION:", action)
        self.action_based_movement(action)
        self.timestep += 1
