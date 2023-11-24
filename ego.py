import carla
import random
import math

class Ego():
    def __init__(self, world, bp, spawn_point):
        self.world = world
        self.vehicle = None
        self.bp = bp 
        while(self.vehicle is None):
            self.vehicle = world.try_spawn_actor(self.bp, spawn_point)
        self.spectator = world.get_spectator()

    def update_spectator(self):
        new_yaw = math.radians(self.vehicle.get_transform().rotation.yaw)
        spectator_transform =  self.vehicle.get_transform()
        spectator_transform.location += carla.Location(x = -10*math.cos(new_yaw), y= -10*math.sin(new_yaw), z = 5.0)
        
        self.spectator.set_transform(spectator_transform)
        self.world.tick()

    def destroy(self):
        self.vehicle.destroy()