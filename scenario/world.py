import lgsvl 
import os  

class World(object):

    def __init__(self, user_map): 
        self.map = user_map
        self.sim = None 

    def load(self):
        self.sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
        if self.sim.current_scene == self.map:
            self.sim.reset() 
        else:
            self.sim.load(self.map)

    def set_rain(self, percentage):
        self.sim.weather.rain = percentage 

    def set_time_of_day(self, daytime):
        self.sim.set_time_of_day(daytime, fixed=True)
    
    def reset(self):
        self.sim = None

    def set_road_type(self, road_type):
        self.road_type = road_type 

    def set_road_friction(self, friction):
        self.road_friction = friction 
    

