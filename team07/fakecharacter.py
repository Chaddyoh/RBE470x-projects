# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld

sys.path.insert(0, '../bomberman')

class FakeCharacter(CharacterEntity):
    
    def do(self, wrld):
        sensed_world = SensedWorld.from_world(wrld)
        character = sensed_world.me(self)
        character.place_bomb()
        (new_world, events) = sensed_world.next()
        print("kachow")
        print("sensed world: ", (new_world.printit(), events))