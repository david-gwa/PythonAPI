from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import ServerActorPool 

MAX_EGO_SPEED = 100 # 11.18 #unit m/s (40 km/h, 25 mph)
MAX_FOLLOWING_DISTANCE = 100 # 10 # The maximum distance the EGO should be from the POV 
MAX_NPC_SPEED =  100 #8.94 #unit m/s (32 km/h, 20 mph)

TTC_TRIGGER_NPC_CUTIN_SCENARIO = ["TTCTriggerNpcCutIN"]

class TTCTriggerNpcCutIN(BasicScenario):
    
    #TODO: parameters pass-in, not hard-code 
    def __init__(self, ego, agents,config = None,sim=None):
        self.sim=sim
        self._npc_drive_distance = 20000 
        self._npc1_speed = 20 #max spd unit km/h
        self._npc2_speed = 20 #max spd unit km/h
        self._time_to_collision1 = 15 # unit s
        self._time_to_collision2 = 15 # unit s
        super(TTCTriggerNpcCutIN, self).__init__("TTCTriggerNpcCutIN", ego, agents,simendtime = 120,config=config)            
        input("TTCTriggerNpcCutIN Scenario hold on...")

    def _initialize_actors(self,config):
        for actor in config.other_actors:
            npc_transform = self.sim.map_point_on_lane(actor.transform.position)
            #TODO: set npc names by configure or command line input
            npc = ServerActorPool.request_new_npc(actor.rolename, npc_transform)
            self.other_actors.append(npc)

        ServerDataProvider.register_actors(self.other_actors)  

        self.logger.log.debug("initial actor in TTCTriggerNpcCutIN scenario done")
        for actor in self.other_actors:
            log = actor.state.transform.position 
            self.logger.log.info("npc %s initial position (%4.2f, %4.2f, %4.2f)" % (actor.name, log.x, log.y, log.z))
            self.logger.log.info("npc %s max speed: %s " % (actor.name, MAX_NPC_SPEED))

    def _create_behavior(self):
        npc_travel_first_half =  py_trees.composites.Parallel("npc csc first half",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        
        npc_navigation1 = FollowClosestLane(self.other_actors[0], self._npc1_speed)
        npc_navigation2 = FollowClosestLane(self.other_actors[1], self._npc2_speed)
        npc_ttc_trigger1 =  InTimeToArrivalToVehicle( self.other_actors[0],self.ego, self._time_to_collision1, comparison_operator=operator.lt, name="TimeToArrival")
        npc_ttc_trigger2 =  InTimeToArrivalToVehicle( self.other_actors[1],self.ego, self._time_to_collision2, comparison_operator=operator.lt, name="TimeToArrival1")
        
        npc_travel_first_half.add_child(npc_navigation1)
        npc_travel_first_half.add_child(npc_navigation2)
        npc_travel_first_half.add_child(npc_ttc_trigger1)
        npc_travel_first_half.add_child(npc_ttc_trigger2)

        npc_travel_second_half =  py_trees.composites.Parallel("npc cut_in second_half",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)   
        npc_change_lane1 = NpcChangeLane(self.other_actors[0], True)
        npc_change_lane2 = NpcChangeLane(self.other_actors[1], True)
        npc_travel_second_half.add_child(npc_change_lane1)
        npc_travel_second_half.add_child(npc_change_lane2)
        sequence = py_trees.composites.Sequence("npc cut_in behavior")
        sequence.add_child(npc_travel_first_half)
        sequence.add_child(npc_travel_second_half)
        sequence.add_child(npc_navigation1)
        sequence.add_child(npc_navigation2)

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

   
