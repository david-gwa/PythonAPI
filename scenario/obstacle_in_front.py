
from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import * 

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
MAX_FOLLOWING_DISTANCE = 10 # The maximum distance the EGO should be from the POV 
MAX_NPC_SPEED =  8.94 # (32 km/h, 20 mph)

OBSTACLE_IN_FRONT_SCENARIO = ["ObstacleInFront"]

class ObstacleInFront(BasicScenario):
    
    #TODO: parameters pass-in, not hard-code 
    def __init__(self, ego, agents):
        self.obstacles = [] 
        self.obstacle_spawn_trans = []
        super(ObstacleInFront, self).__init__("ObstacleInFront", ego, agents)
        input("obstacle in front scenario hold on...")

    def _initialize_actors(self,config):
        for actor in self.other_actors:
            self.obstacle_spawn_trans.append(actor.transform) 
            ServerActorPool._sim.remove_agent(actor)
        for trans in self.obstacle_spawn_trans:
            ped = ServerActorPool.request_new_obstacle("RoadBlocker2", trans)
            self.obstacles.append(ped)
        self.logger.log.debug("initial obstacles in ObstacleInFront scenario done")

    def _create_behavior(self):
        ego_position =  self.ego.state.transform.position 
        min_x = ego_position.x - 20 
        max_x = ego_position.x + 20 
        min_z = ego_position.z - 10
        max_z = ego_position.z + 10
        obs_in_trigger_region = InTriggerRegion(self.other_actors[0], min_x, max_x, min_z, max_z)
        sequence = py_trees.composites.Sequence("ObstacleInFront behavior")
        sequence.add_child(obs_in_trigger_region)
        return sequence 


    def _create_test_criteria(self):
        criteria = []
        ego_collision_criterion = CollisionTest(self.ego)
        ego_maxspeed_criterion = MaxVelocityTest(self.ego, MAX_EGO_SPEED)

        criteria.append(ego_collision_criterion)
        criteria.append(ego_maxspeed_criterion)

        return criteria 

   