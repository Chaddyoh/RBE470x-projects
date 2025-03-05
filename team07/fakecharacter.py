# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from colorama import Fore

sys.path.insert(0, '../bomberman')

class FakeCharacter(CharacterEntity):
    
    def do(self, wrld):
        sensed_world = SensedWorld.from_world(wrld)
        
        character = sensed_world.me(self)
        character.move(1, 1)
        (new_world, events) = sensed_world.next()
        print(Fore.GREEN + f"just moved:  {(new_world.printit(), events)}")
        (new_world, events) = new_world.next()
        print(Fore.GREEN + f"just moved:  {(new_world.printit(), events)}")
        (new_world, events) = new_world.next()
        print(Fore.GREEN + f"just moved:  {(new_world.printit(), events)}")

        # character = new_world.me(self)
        # character.move(0, 0)
        # character.place_bomb()
        (new_world, events) = new_world.next()
        print(Fore.MAGENTA + f"splaced bomb:  {(new_world.printit(), events)}")

        # character = new_world.me(self)
        # character.move(1,-1)
        (new_world, events) = new_world.next()
        print("moved again: ", (new_world.printit(), events))