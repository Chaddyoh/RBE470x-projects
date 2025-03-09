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
    weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1,1,1,1]
    bomb_loc = None
    bomb_placed_time = 0
    is_training = True
    epsilon = 0.1

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


    def get_neighbors(self, wrld, current) -> list[tuple]: # Returns a list of tuples of the surrounding empty nodes. Assumes the exit node is empty

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
        if path:
            current_node = path[0]
            next_node = path[1]

            dx = next_node[0] - current_node[0]
            dy = next_node[1] - current_node[1]

            if wrld.empty_at(self.x + dx, self.y + dy):
                self.move(dx, dy)
            else:
                #print(Fore.RED + f"I placed a bomb :D")
                self.place_bomb()
                self.bomb_placed_time = self.timestep
        elif self.state == Enum.BOMBING:
            #print("x:", self.x, "\ty:", self.y)
            if wrld.empty_at(self.x-1, self.y-1):
                self.move(-1, -1)
            elif wrld.empty_at(self.x+1, self.y-1):
                self.move(1, -1)
            elif wrld.empty_at(self.x-1, self.y+1):
                self.move(-1, 1)
            elif wrld.empty_at(self.x+1, self.y+1):
                self.move(1, 1)
            elif wrld.empty_at(self.x-1, self.y):
                self.move(-1, 0)
            elif wrld.empty_at(self.x+1, self.y):
                self.move(1, 0)
            elif wrld.empty_at(self.x-1, self.y):
                self.move(0, -1)
            elif wrld.empty_at(self.x+1, self.y):
                self.move(0, 1)
        else:
            pass
        #print("cry")

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

    def is_in_blast_radius(self, wrld, state = None):
        if not state:
            state = (self.x, self.y)
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            return state[0] == bomb_loc[0] and abs(state[0] - bomb_loc[0]) < 5 and state[1] == bomb_loc[1] and abs(state[1] - bomb_loc[1]) < 5
        else:
            return False

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



    def pick_best_action(self, wrld, state):
        #actions = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE', 'bomb'] # all player moves
        actions = self.get_walkable_actions(wrld, state)
        max_Q = -math.inf
        # Max Q(s', a') part
        best_action = ""
        for a in actions:
            new_sensed_wrld, (dx, dy) = self.next_sensed_wrld(wrld, a)
            Q_next_state = 0
            #if not new_sensed_wrld.me(self):
            #    prev_character = list(wrld.characters.values())[0][0]
            #    prev_character.x += dx
            #    prev_character.y += dy
            #    # add movement for character (x,y)
            #    new_sensed_wrld.characters[78] = [prev_character]
            delta = self.action_to_delta(a)
            new_features = self.feature_calculator(new_sensed_wrld, delta)
            print("NEW FEATURES: ", new_features)
            for i in range(len(self.weights)):
                Q_next_state += self.weights[i] * new_features[i]
                if Q_next_state > max_Q:
                    max_Q = Q_next_state
                    best_action = a
            print(f"Q({a}) = {Q_next_state}")
            print(f"MAX-Q({best_action}) = {max_Q}")
            return best_action

    def get_walkable_actions(self, wrld, state):
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
        }
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                if (state[0]+dx >=0) and (state[0]+dx < wrld.width()) and (state[1]+dy >=0) and (state[1]+dy < wrld.height()):
                    if wrld.empty_at(state[0]+dx, state[1]+dy) or wrld.exit_at(state[0]+dx, state[1]+dy) or not wrld.bomb_at(state[0]+dx, state[1]+dy):
                        if not (dy == 0 and dx ==0):
                            walkable_actions.append(direction_to_action[(dx, dy)])
        if not wrld.bombs:
            walkable_actions.append("bomb")
        return walkable_actions

    def feature_calculator(self, wrld, a):
        """The sole job of this is to evaluate its setup"""
        f1 = 0
        f2 = 0
        f3 = 0
        f4 = 0
        f5 = 0
        f6 = 0
        f7 = 0
        f8 = 0
        f9 = 0
        f10 = 0
        f11 = 0
        f12 = 0
        f13 = 0
        f14 = 0
        f15 = 0
        f16 = 0
        f17 = 0
        f18 = 0
        f19 = 0
        f20 = 0
        if isinstance(wrld, SensedWorld):
            character = wrld.me(self)
            #print("FEATURE CALULATOR STAT: ", character.x, character.y)
            if type(a) == tuple:
                state = (character.x+a[0], character.y+a[1])
            else:
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
        exit_dist_x =(exit_loc[0]-state[0])
        exit_dist_y= exit_loc[1]-state[1]

        #print("LEN EXIT PATH: ", exit_dist)
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
            bomb_dist = 0
            bomb_dangerzone = 0

        if monster_count and exit_path:
            monster_exit_dist = self.heuristic((closest_monster.x, closest_monster.y), exit_path[1])
            try:
                angle = math.acos(((monster_exit_dist**2 - exit_euc_dist**2 - closest_monster_dist**2) / (-2 * exit_euc_dist * closest_monster_dist)))
            except:
                angle = 0

        """f1=monster_dist, f2=monster_count, f3=exit_dist, f4=bomb_dist, f5=in_bomb_path + bomb_time_existing,"""
        #print(wrld.height(), exit_dist_y, wrld.width(), exit_dist_x)
        f1 = wrld.height()/(exit_dist_y+1 ) # the larger the average monster distance, the smaller the f
        f2 = wrld.width()/(exit_dist_x+1)   # 1.0, 0.5, or 0.0
        f3 =1/((exit_dist +1)**2)
    #     # longer the distance, the smaller the f
    #     if wrld.bombs:
    #         f4 =(bomb_loc[0]-state[0]/wrld.width())
    #         f10=(bomb_loc[1]-state[1]/wrld.height())
    #         f5 = 1 / (bomb_exists_time + 1)
    #         f11 =(bomb_loc[0]-state[0])*(bomb_loc[1]-state[1]) 
    #         f14 = bomb_loc[0]
    #         f15 = bomb_loc[1]
    #         f16 = bomb_dangerzone
    #    # longer the time goes on, the larger f5 gets
    #     if monster_count:
    #         f6 = 1/ (closest_monster_dist + 1)
    #     if wrld.explosion_at(character.x, character.y):
    #         f7 = 1
    #     f8 =1/ (1 + len(self.get_walkables(wrld, (character.x, character.y)))) #if in corner
    #     if monster_count and exit_path:
    #         if angle > 0: # f9 = 1 if there is an angle between monster and next part of path
    #             f9 = 1
    #     f12= (state[0]-1)/(wrld.width())
    #     f13=(state[1]-1)/(wrld.height())
    #     f18=state[0]
    #     f19=state[1]        

        features = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12,f13,f14,f15,f16,f17,f18,f18,f19,f20]
        # Normalize features
        for i in range(len(features)):
            features[i] = features[i]/sum(features)
        return features


    def reward_calculator(self, wrld, state):
        x, y = state
        #print("CALCULATING REWARD: ", state)
        #print("WORLD GRID: ", wrld.width(), wrld.height())
        reward = 0
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
        if x < wrld.width() and y<wrld.height() and y >= 0 and x >= 0:
            #print("VIABLE CONDITION")
            if wrld.empty_at(x, y):
                reward+=-6
                #print("REWARD FOR EMPTY CELL")
            if wrld.exit_at(x, y):
                reward+= 500000
                #print("REWARD FOR EXIT")
            if wrld.wall_at(x, y) or x > wrld.width() or x<0 or y<0 or y > wrld.height():
                reward+= -5
                #print("REWARD FOR WALL AT")
            if wrld.bomb_at(x, y):
                reward+= -8
                #print("REWARD FOR BOMB AT CURRENT LOC")
            if wrld.bombs:
                reward+= 100
                #print("REWARD FOR BOMB")
                if bomb_loc[0] == state[0] or bomb_loc[1] == state[1]:
                    reward+= -15
                    #print("REWARD FOR BOMB IN CARDINAL")
            if wrld.monsters_at(x, y):
                reward+= -10
                #print("REWARD FOR MONSTER AT")

            if wrld.explosion_at(x, y):
                reward+= -50
                #print("SYM DIED TO EXPLOSION")
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
            #print("FROM:", old_pos, ", GIVEN: ", action, "CHAR. MOVED: ", character.x, character.y, " NEXT POS: ", character.nextpos())
        sensed_world, events = wrld.next()
        print("EVENTS FROM NEXT SENSED WORLD: ", events)
        return sensed_world, (dx, dy)

    def action_based_movement(self, a):
        a = self.action_to_delta(a)
        if a == "bomb":
            self.place_bomb()
            self.move(0, 0)

        #print("Movement time ! ", action)
        else:
            (dx, dy) = a
            self.move(dx, dy)

    def exploration_function(self, utility, num_tries):
        k = 0.5
        return utility + k / (num_tries +1)

    # def q_learning(self, sensed_wrld, state):
    #     alpha = 0.4
    #     gamma = 0.9
    #     r=0
    #     q = 0

    #     #actions = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE', 'bomb'] # all player moves
    #     actions = self.get_walkable_actions(sensed_wrld, state)
    #     #print("STATE: ", state, "WALKABLE ACTIONS: ", actions)
    #     features = self.feature_calculator(sensed_wrld)
    #     #print("--------------------------------------")
    #     max_action =''
    #     for i in range(len(self.weights)):
    #         q += self.weights[i] * features[i]
    #     max_Q = -math.inf
    #     for a in actions:
    #         r=0
    #         new_sensed_wrld, (dx, dy) = self.next_sensed_wrld(sensed_wrld, a)
    #         if new_sensed_wrld.me(self):
    #             Q_next_state = 0
    #             new_features = self.feature_calculator(new_sensed_wrld)
    #             for i in range(len(self.weights)):
    #                 Q_next_state += self.weights[i]* new_features[i]
    #             if Q_next_state > max_Q:
    #                 max_Q = Q_next_state
    #                 max_action = a
    #                 r = self.reward_calculator(new_sensed_wrld, (state[0]+dx, state[1]+dy))
    #                 delta = (r + gamma * max_Q) - q

    #             #print("ACTION: ", a, "Delta Pos. : ", dx, dy, "Features: ", new_features, "Q"+"("+a+"): ", Q_next_state, "R: ", r)
    #         else:
    #             delta = 0
    #     for i in range(len(self.weights)):
    #         self.weights[i] = self.weights[i] + alpha * delta * features[i]
    #     #print("Weigths: ", self.weights)
    #     print(delta,"=",r, "+",max_Q, "-", q )
    #     print(features)
    #     print(self.weights)
    #     #print("MAX PICK: ", max_action, max_Q)
    #     if max_action == '':
    #         return 0
    #     else: 
    #         return max_action

    def action_to_delta(self, a=None, delta=None):
        action_dictionary = {
            'N': (0, -1),
            'NW': (-1, -1),
            'W' : (-1, 0),
            'SW' : (-1, 1),
            'S': (0, 1),
            'SE': (1, 1),
            'E': (1, 0),
            'NE': (1, -1),
            'bomb':'bomb'
        }
        if a:
            return action_dictionary[a]
        if delta:
            for key, val in action_dictionary.items():
                if val == delta:
                    return key
            return None


    def q_learning(self, sensed_wrld, state):
        alpha = 0.4
        gamma = 0.9
        print("--------------------------STARTING Q--------------------------------")
        # Get possible actions
        actions = self.get_walkable_actions(sensed_wrld, state)
        print("get walkable: ", actions)
        # Compute Q-values for all actions in current state
        q_values = {}
        for a in actions:
            #print("LOOP a: ", a)
            delta = self.action_to_delta(a)
            #print("DELT: ",delta)
            #print("CURRENT Q-STATE: ", state)
            features = self.feature_calculator(sensed_wrld, delta)  # Features depend on action
            q_values[a] = sum(self.weights[i] * features[i] for i in range(len(self.weights)))

        print("All q-values for current: ", q_values)

        # Select action (e.g., max Q-value; add exploration like epsilon-greedy if needed)
        max_action = max(q_values, key=q_values.get)
        q = q_values[max_action]
        print("Current q-val based on max: ", q)
        features = self.feature_calculator(sensed_wrld, max_action)
        print("Features of q: ", features)

        # Simulate the action
        new_sensed_wrld, (dx, dy) = self.next_sensed_wrld(sensed_wrld, max_action)
        next_state = (state[0] + dx, state[1] + dy)
        r = self.reward_calculator(new_sensed_wrld, next_state)
        print("REWARD FOR NEXT STATE: ",r )

        print("Compute the max q-val in the next state: ") 

        # Compute max Q-value in next state
        if new_sensed_wrld.me(self):
            next_actions = self.get_walkable_actions(new_sensed_wrld, next_state)
            print("Next Walkable Actions:", next_actions)
            max_Q_next = -math.inf
            for next_a in next_actions:
                delta = self.action_to_delta(next_a)
                next_features = self.feature_calculator(new_sensed_wrld, delta)
                print(f"Features of next action {next_a}: {next_features}")
                Q_next = sum(self.weights[i] * next_features[i] for i in range(len(self.weights)))
                max_Q_next = max(max_Q_next, Q_next)
                print(f"Q({next_a}): {Q_next}")
        else:
            max_Q_next = 0  # Terminal state
        
        # Compute TD error
        delta = r + gamma * max_Q_next - q
        
        # Update weights
        for i in range(len(self.weights)):
            self.weights[i] += alpha * delta * features[i]
        print("WEIGTHS: ", self.weights)
        print(f"Max Q: {max_Q_next}, Max action: {max_action}")

        if random.random() < self.epsilon:
            return random.choice(actions)
            print("RANDOM ACTION CHOSEN")
        else:
            return max_action    
    def training(self, wrld):
        for iteration in range(3):
            # Make new world for each iteration
            sensed_world = SensedWorld.from_world(wrld) #  Current State
            finished_game = False
            character = sensed_world.me(self)
            while character:
                state = (character.x, character.y)
                #character = sensed_world.me(self) #Copy current character to get location
                print("Current Character Location: ",character.x, character.y )
                next_action = self.q_learning(sensed_world, state) # Run q-learning which will update weights
                print(f"NEXT ACTION FROM Q: {next_action}")
                #print("SENSED WRLD: ", sensed_world)
                #print("EVENTS: ", sensed_world.events)
                sensed_world, (dx, dy) = self.next_sensed_wrld(sensed_world, next_action)
                print("STATE: ", state, "NEW DW: ", (dx, dy))
                character = sensed_world.me(self)
            print(f"Weights are {self.weights} for iteration {iteration}")
        return False

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
            #print("update path to monster")
            path_to_monster = self.plan_path(wrld, start, monster_loc)

        #print("path to monster: ", path_to_monster)

        path = self.plan_path(wrld, start, self.exit)
        if path:
            by_wall = self.is_by_wall(wrld, path)
        if path_to_monster:
            trapped_with_monster = self.trapped_with_monster(wrld, path, path_to_monster)
            #print("trapped with monster: ", trapped_with_monster)

        in_range_of_bomb = self.is_in_blast_radius(wrld)
        # TODO: use time left in bomb to determine actions
        #self.q_learning(wrld, self.state)
        #print(f"state: {self.state}")        
        if self.is_training:
            self.is_training = self.training(wrld)

        action = self.pick_best_action(wrld, (self.x, self.y))
        print()
        #print("ACTION:", action)
        self.action_based_movement(action)

        # match self.state:
        #     case Enum.TRAVELING:
        #         if monster_loc and trapped_with_monster:
        #             self.training(wrld)
        #             action = self.pick_best_action(wrld, (self.x, self.y))
        #             self.action_based_movement(action)
        #             self.state = Enum.FLEEING
        #         elif by_wall:
        #             if bomb_exists:
        #                 state = Enum.WAITING
        #             else:
        #                 # Move to wall, then place Bomb
        #                 self.color_path(path)
        #                 self.next_step(wrld, path)
        #                 # Switch States
        #                 self.state = Enum.BOMBING
        #         else:
        #             # A* walking
        #             self.color_path(path)
        #             self.next_step(wrld, path)
        #     case Enum.BOMBING:
        #         if monster_loc and trapped_with_monster:
        #             self.training(wrld)
        #             action = self.pick_best_action(wrld, (self.x, self.y))
        #             self.action_based_movement(action)
        #             self.state = Enum.FLEEING
        #         elif not in_range_of_bomb: # Player in danger of explosion
        #             self.state = Enum.WAITING
        #         else:
        #             self.next_step(wrld)
        #     case Enum.FLEEING:
        #         if not monster_loc:
        #             if bomb_exists: # Bomb is on map
        #                 self.state = Enum.BOMBING
        #             else:
        #                 self.color_path(path)
        #                 self.next_step(wrld, path)
        #                 self.state = Enum.TRAVELING
        #         else:
        #             action = self.pick_best_action(wrld, (self.x, self.y))
        #             self.action_based_movement(action)
        #             # self.avoid_monster(wrld)
        #     case Enum.WAITING:
        #         if monster_loc and trapped_with_monster:
        #             # MONSTER AHH
        #             # self.avoid_monster(wrld)
        #             self.training(wrld)
        #             action = self.pick_best_action(wrld, (self.x, self.y))
        #             self.action_based_movement(action)
        #             self.state = Enum.FLEEING
        #         elif not bomb_exists:
        #             # no bomb
        #             self.state = Enum.TRAVELING
        #         else:
        #             self.move(0,0)


        self.timestep += 1
