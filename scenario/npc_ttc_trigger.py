from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import ServerActorPool 

MAX_EGO_SPEED = 100 # 11.18 #unit m/s (40 km/h, 25 mph)
MAX_FOLLOWING_DISTANCE = 100 # 10 # The maximum distance the EGO should be from the POV 
MAX_NPC_SPEED =  100 #8.94 #unit m/s (32 km/h, 20 mph)

NPC_TTC_TRIGGER_SCENARIO = ["NpcTTCTrigger"]

class NpcTTCTrigger(BasicScenario):
    
    #TODO: parameters pass-in, not hard-code 
    def __init__(self, ego, agents,config = None,sim=None):
        self.sim=sim
        self._npc_drive_distance = 2000 
        self._npc_speed = 10 #max spd unit km/h
        self._time_to_collision = 5 # unit s
        super(NpcTTCTrigger, self).__init__("NpcTTCTrigger", ego, agents,simendtime = 120,config=config)            
        input("NpcTTCTrigger Scenario hold on...")

    def _initialize_actors(self,config):
        for actor in config.other_actors:
            npc_transform = self.sim.map_point_on_lane(actor.transform.position)
            #TODO: set npc names by configure or command line input
            npc = ServerActorPool.request_new_npc(actor.rolename, npc_transform)
            self.other_actors.append(npc)

        ServerDataProvider.register_actors(self.other_actors)  

        self.logger.log.debug("initial actor in NpcTTCTrigger scenario done")
        for actor in self.other_actors:
            log = actor.state.transform.position 
            self.logger.log.info("npc %s initial position (%4.2f, %4.2f, %4.2f)" % (actor.name, log.x, log.y, log.z))
            self.logger.log.info("npc %s max speed: %s " % (actor.name, MAX_NPC_SPEED))

    def _create_behavior(self):
        npc_travel_first_half =  py_trees.composites.Parallel("npc csc first half",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        npc_navigation = FollowClosestLane(self.other_actors[0], self._npc_speed)
        npc_ttc_trigger =  InTimeToArrivalToVehicle( self.other_actors[0],self.ego, self._time_to_collision, comparison_operator=operator.lt, name="TimeToArrival")
        npc_travel_first_half.add_child(npc_navigation)
        npc_travel_first_half.add_child(npc_ttc_trigger)

        npc_travel_second_half =  py_trees.composites.Parallel("npc cut_in second_half",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)   
        npc_change_lane = NpcChangeLane(self.other_actors[0], False)
        npc_travel_second_half.add_child(npc_change_lane)
        sequence = py_trees.composites.Sequence("npc cut_in behavior")
        sequence.add_child(npc_travel_first_half)
        sequence.add_child(npc_travel_second_half)
        sequence.add_child(npc_navigation)

        return sequence 


    def _create_test_criteria(self):
        criteria = []
        ego_collision_criterion = CollisionTest(self.ego)
        ego_maxspeed_criterion = MaxVelocityTest(self.ego, MAX_EGO_SPEED)
        npc_collision_criterion = CollisionTest(self.other_actors[0])
        npc_maxspeed_criterion = MaxVelocityTest(self.other_actors[0], MAX_NPC_SPEED)

        criteria.append(ego_collision_criterion)
        criteria.append(ego_maxspeed_criterion)
        criteria.append(npc_maxspeed_criterion)
        criteria.append(npc_collision_criterion)

        return criteria 

   
