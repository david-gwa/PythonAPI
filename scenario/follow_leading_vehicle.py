
from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import ServerActorPool 


FOLLOW_LEADING_VEHICLE_SCENARIO = ["FollowLeadingVehicle"]

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
MAX_NPC_SPEED = 60  # 8.94 # (32 km/h, 20 mph)
MAX_NPC_ROTATION = 5 #deg/s
SPEED_VARIANCE = 4 # Without real physics, the calculation for a rigidbody's velocity is very imprecise
MAX_FOLLOWING_DISTANCE = 10 # The maximum distance the EGO should be from the POV 

class FollowLeadingVehicle(BasicScenario):

    def __init__(self, ego, agents):
        self._npc_spawn_location = None 
        self._npc_transform = None 
        self._npc_speed = 40
        self._npc_stop_in_front_intersection = 20
        self._npc_max_brake = 1.0 
        super(FollowLeadingVehicle, self).__init__("FollowVehicle", ego, agents)
        ServerDataProvider.register_actors(self.other_actors)  
        input("FollowLeadingVehicle Scenario hold on...")

    def _initialize_actors(self,config):
        self.logger.log.debug("size of other_acters: %d" % len(self.other_actors))
        for actor in self.other_actors:
            log = actor.state.transform.position 
            self.logger.log.info("npc %s initial position (%4.2f, %4.2f, %4.2f)" % (actor.name, log.x, log.y, log.z))
            self.logger.log.info("npc %s max speed: %s " % (actor.name, MAX_NPC_SPEED))

    def _create_behavior(self):
        npc_navigation = FollowClosestLane(self.other_actors[0], self._npc_speed)
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(npc_navigation)
        return sequence

    def _create_test_criteria(self):
        criteria = []
        ego_collision_criterion = CollisionTest(self.ego)
        npc_collision_criterion = CollisionTest(self.other_actors[0]) 
        ego_maxspeed_criterion = MaxVelocityTest(self.ego, MAX_EGO_SPEED)
        npc_maxspeed_criterion = MaxVelocityTest(self.other_actors[0], MAX_NPC_SPEED)
        npc_maxangularspeed_criterion = MaxAngularVelocityTest(self.other_actors[0], MAX_NPC_ROTATION)
        #TODO: separaton distance criterion only trigger at final time
        separation_distance_criterion = SeparationDistanceTest(self.ego, self.other_actors[0], MAX_FOLLOWING_DISTANCE)
        criteria.append(ego_collision_criterion)
        criteria.append(npc_collision_criterion)
        criteria.append(ego_maxspeed_criterion)
        criteria.append(npc_maxspeed_criterion)
        criteria.append(npc_maxangularspeed_criterion)
        criteria.append(separation_distance_criterion)
        return criteria 



