
from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import ServerActorPool 

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
MAX_NPC_SPEED =  3 

PEDESTRAIN_STILL_IN_FRONT = ["PedestrainStillInFront"]

class PedestrainStillInFront(BasicScenario):
    
    def __init__(self, ego, agents):
        self.pedestrains = []
        self._pedestrain_speed = 1.0 
        self.pedestrain_spawn_trans = []
        super(PedestrainStillInFront, self).__init__("PedestrainStillInFront", ego, agents)
        input("PedestrainStillInFront Scenario hold on...")

    def _initialize_actors(self,config):
        for actor in self.other_actors:
            self.pedestrain_spawn_trans.append(actor.transform) 
            ServerActorPool._sim.remove_agent(actor)
        for trans in self.pedestrain_spawn_trans:
            ped = ServerActorPool.request_new_pedestrain("Bob", trans)
            self.pedestrains.append(ped)
        self.logger.log.debug("initial pedestrains in Pedestrain still in front of eg scenario done")

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence("PedestrainStillInFront behavior")
        stand_still = PedestrainStandStill(self.pedestrains[0], 15.0)
        sequence.add_child(stand_still)

        return sequence 


    def _create_test_criteria(self):
        criteria = []
        ego_collision_criterion = CollisionTest(self.ego)
        ego_maxspeed_criterion = MaxVelocityTest(self.ego, MAX_EGO_SPEED)
        npc_collision_criterion = CollisionTest(self.pedestrains[0])
        npc_maxspeed_criterion = MaxVelocityTest(self.pedestrains[0], MAX_NPC_SPEED)

        #criteria.append(ego_collision_criterion)
        #criteria.append(ego_maxspeed_criterion)
        criteria.append(npc_maxspeed_criterion)
        criteria.append(npc_collision_criterion)

        return criteria 

   
