import lgsvl
import py_trees
from scenario.logger import *
from scenario.scenario_manager import Scenario
from scenario.server_data_provider import ServerDataProvider, ServerActorPool
from scenario.actor_pos import ActorPos


class BasicScenario(object):

    def __init__(self, name, ego_vehicles,
     agents=[], 
	 simendtime = 30,
    config =None, world=None, debug_mode=False, terminate_on_failure=False, criteria_enable=False):
        self.name = name 
        self.timeout = simendtime
        self.ego = ego_vehicles
        self.ego_vehicles=ego_vehicles
        self.logger = logger(self.__class__.__name__) #to get the derived subclass 
        self.logger.set_output_file()
        self.other_actors = agents
        self._initialize_actors(config)
        behavior = self._create_behavior()
        criteria = self._create_test_criteria()
        behavior_seq = py_trees.composites.Sequence()
        behavior_seq.add_child(behavior)
        self.scenario = Scenario(behavior_seq, criteria, self.name, self.timeout)

    def _initialize_actors(self,config):
        raise NotImplementedError("this funcion suppose to implement by all scenarios")

    def _create_behavior(self):
        raise NotImplementedError("this funcion suppose to implement by all scenarios")
    
    def _create_test_criteria(self):
        raise NotImplementedError("this funcion suppose to implement by all scenarios") 

    def remove_all_actors(self):
        """
        Remove all actors
        """
        print("Remove all actors")