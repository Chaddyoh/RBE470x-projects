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
import os
from termcolor import colored

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
    weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1]
    bomb_loc = None
    bomb_placed_time = 0
    is_training = True
    epsilon = 0.1
    ddx = 0
    ddy = 0
    dwallcount =24
    states_visited =[]
    batch_wins ={}
    winrate=0


    def locate_exit(self, wrld) -> tuple: # Returns X,Y tuple for exit
        for x_coordinate in range(wrld.width()):
            for y_coordinate in range(wrld.height()):
                if wrld.exit_at(x_coordinate, y_coordinate):
                    return (x_coordinate, y_coordinate)


    def check_for_monster(self, wrld, current) -> tuple:
        global monsters
        monsters_dict = {}
        for dx in [-4,-3,-2,-1,0,1,2,3, 4]:
            if (current[0]+dx >=0) and (current[0]+dx < wrld.width()):
                for dy in [-4,-3,-2,-1,0,1,2,3,4]:
                    if (current[1]+dy >=0) and (current[1]+dy < wrld.height()):
                        monster_square = wrld.monsters_at(current[0]+dx, current[1]+dy)
                        if monster_square:
                            monsters_dict[0] = (current[0]+dx, current[1]+dy)
        return monsters_dict


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
                xpos = state[0]+dx
                ypos = state[1]+dy
                #print("STATE: ", state," ACTION: ", self.action_to_delta((dx, dy)), " DXDY: ", (dx, dy), " POS:", state[0]+dx, state[1]+dy)
                if (xpos >=0) and (ypos >=0) :
                    if  (xpos< wrld.width()) and (ypos < wrld.height()):
                        if wrld.empty_at(xpos, ypos) or wrld.exit_at(xpos, ypos) or (dx, dy) == (0,0):
                            if not wrld.wall_at(xpos, ypos):
                                if wrld.bombs:
                                    bomb_obj = list(wrld.bombs.values())[0] 
                                    bomb_loc = (bomb_obj.x, bomb_obj.y)
                                    if xpos != bomb_loc[0] and ypos != bomb_loc[1]:
                                        walkable_actions.append(self.action_to_delta((dx, dy)))
                                else:
                                    walkable_actions.append(self.action_to_delta((dx, dy)))
        if not wrld.bombs:
            walkable_actions.append("bomb")
        if not walkable_actions:
            walkable_actions.append("stay")
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
        mmanhattan_dist = 0
        f1 = manhattan_dist / max_manhattan
        f12=0
        f13=0
        monsters = self.check_for_monster(wrld, next_state)
        if wrld.monsters:
            monster = self.check_for_monster(wrld, next_state)
            for m, p in monster.items():
                mmanhattan_dist = abs(p[0] - next_state[0]) + abs(p[1] - next_state[1])
                f12 = p[0]-next_state[0]
                f13=p[1]-next_state[1]

            #print(monster)
            f2=mmanhattan_dist/max_manhattan
        else:
            f2 = 0
            

        
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0] 
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            bomb_timer = bomb_obj.timer
            bomb_dist = abs(bomb_loc[0] - next_state[0]) + abs(bomb_loc[1] - next_state[1])
            f3 = bomb_dist / max_manhattan  # Normalized distance to bomb
            
            blast_radius = 4
            in_blast_radius = ((abs(bomb_loc[0] - next_state[0]) <= blast_radius and bomb_loc[1] == next_state[1]) or (abs(bomb_loc[1] - next_state[1]) <= blast_radius and bomb_loc[0] == next_state[0]))
            f4 = 1 if in_blast_radius else 0  # Binary feature
            
            f5 = 1 / (bomb_timer + 1)  # Inverse timer, 0 < f5 <= 1
        else:
            f3 = 0  
            f4 = 0  
            f5 = 0  
        
        f6 = 1 if a == 'stay' else 0 
        f7 = 1 if a == 'bomb' else 0 
        
        # Feature 8: Normalized number of walkable actions
        walkable_actions = self.get_walkable_actions(wrld, next_state)  # List of valid actions
        f8 = len(walkable_actions) / 9.0  # Normalize by max 4 directions
        
        # Feature 9: Explosion at next state
        if wrld.explosion_at(current_state[0], current_state[1]):
            f9=1
        else:
            f9=0
        
        # Feature 10: Bias term
        f10 = 1  # Constant feature for Q-function offset
        f11=1 if next_state not in self.states_visited else 0
        # Return the feature vector
        features = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12 , f13]
        return features

    def reward_calculator(self, wrld, state):
        x, y = state
        
        #print("CALCULATING REWARD: ", state)
        #print("WORLD GRID: ", wrld.width(), wrld.height())
        reward = 0
        exit_loc = self.locate_exit(wrld)
        if (exit_loc[0]-x) > (exit_loc[0] - self.ddx):
            #print("REWARD Getting Closer in the X")
            reward += 2
        if (exit_loc[1]-y) < (exit_loc[1] - self.ddy):
            reward += 5
        if (self.count_walls(wrld)<self.dwallcount):
            reward += 14
        if  not (x == self.ddx and y ==self.ddy):
            reward += 4       
        if wrld.exit_at(x, y):
            reward+= 2500
        if wrld.bombs:
            bomb_obj = list(wrld.bombs.values())[0]
            bomb_loc = (bomb_obj.x, bomb_obj.y)
            if bomb_loc[0] != state[0] and bomb_loc[1] != state[1]:
                reward+=12
        # if not wrld.monsters_at(x, y):
        #     reward+=5
        if not wrld.explosion_at(x, y):
            reward+=2
        if self.check_for_monster(wrld, state):
            monster = self.check_for_monster(wrld, state)
            for m, p in monster.items():
                mmanhattan_dist = abs(p[0] - state[0]) + abs(p[1] - state[1])
                #print("Monster DIST: ", mmanhattan_dist)
                reward +=5*mmanhattan_dist 
        if state not in self.states_visited:
            self.states_visited.append(state)
            reward+=12
        reward-=1
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
        alpha = 0.39
        gamma = 0.9

        actions = self.get_walkable_actions(sensed_wrld, state)
        #print(f"State: {state}, Walkable Actions: {actions}")
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
                next_action = self.action_to_delta(next_a)
                next_features = self.feature_calculator(new_sensed_wrld, next_action)
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
        char_w = 0
        for iteration in range(iterations):
            # Make new world for each iteration
            #self.epsilon = self.epsilon**(iteration/(iterations))
            self.states_visited = []
            sensed_world = SensedWorld.from_world(wrld) #  Current State
            character = sensed_world.me(self)
            self.bwd = 0
            if iteration%100==0:
                self.batch_wins[int(iteration/100)] = int(self.winrate)
                
                if len(self.batch_wins) >=2:
                    self.bwd = (self.batch_wins[(int((iteration)/100)-1)]-self.batch_wins[int(iteration/100)]/2)
                self.batchit =0
                self.winrate=0
                self.char_w=0
            self.batchit+=1
            if self.epsilon>=0:
                self.epsilon = self.epsilon +(-((self.winrate)/(self.epsilon**0.5))+((100-self.winrate)*self.epsilon**4))/100000
            else:
                return False
            while character:
                state = (character.x, character.y)
                self.winrate = int(100*self.char_w/(self.batchit+1))
                if self.itdebug:
                    os.system('clear')
                    print("Weights: ", self.weights)  
                    print(f"\n Training... {100*iteration/iterations:.2f} % complete \n  Epsilon: {self.epsilon:.3f}")
                    print(f"\n BATCH INFORMATION \n  Iteration: {(self.batchit)} \n  Times Won: {self.char_w} \n  Win Rate: {self.winrate:.3f}")
                    print(f" Batch Wins: {self.batch_wins} \n States Visited:  {len(self.states_visited)}, Derivative: {int(self.bwd)}\n")
                if character.y >= 17 and character.x >= 6:
                    self.char_w+=1
                #     print(colored('╔════════════════╗', 'green', 'on_green'))
                #     print(colored('║    WINNING!    ║', 'green', 'on_green'))
                #     print(colored('╚════════════════╝', 'green', 'on_green'))
                # else:
                #     print(colored('╔════════════════╗', 'red', 'on_red'))
                #     print(colored('║    LOSING!     ║', 'red', 'on_red'))
                #     print(colored('╚════════════════╝', 'red', 'on_red'))
                #print(f"\t\t\t\t\tSTATE: ({character.x:>3}, {character.y:>3}), {iteration}")  # Left-justifies "REWARD" with 16 spaces
                next_action = self.q_learning(sensed_world, state) # Run q-learning which will update weights
                sensed_world, (dx, dy) = self.next_sensed_wrld(sensed_world, next_action)
                character = sensed_world.me(self)
            #print(f"Weights are {self.weights} for iteration {iteration}")
        
        return False

    def do(self, wrld):
        # Your code here
        self.exit = self.locate_exit(wrld)
        self.epsilon = 0.45
        self.itdebug = 1
        if self.is_training:
            use_recent_weights = input("Do you want to use the last trained weights? [Y/n]: ")
            if use_recent_weights.lower() == "y": 
                with open("weigths.txt", "r") as wf:
                    lines = wf.readlines()
                    recentw = lines[-1].strip()
                    for i in range(1,len(self.weights)+1):
                        self.weights[-i] = float(lines[-i])
                        print(lines[-i])
                    print("Using the most recent weights: ", self.weights)
                self.is_training = False
            else:
                self.is_training = self.training(wrld, 1000)
                with open("weigths.txt", "a") as wf:
                    wf.write('\n'.join(str(w) for w in self.weights))
                    wf.write('\n')

        action = self.pick_best_action(wrld, (self.x, self.y))
        #print()
        #print("ACTION:", action)
        self.action_based_movement(action)
        self.timestep += 1
