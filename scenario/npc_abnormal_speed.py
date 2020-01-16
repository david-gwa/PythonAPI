import py_trees
from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import ServerActorPool 

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
MAX_FOLLOWING_DISTANCE = 10 # The maximum distance the EGO should be from the POV 
MAX_NPC_SPEED =  8.94 # (32 km/h, 20 mph)
NPC_ABNORMAL_SPEED_SCENARIO = ["NpcAbnormalSpeed"]

class NpcAbnormalSpeed(BasicScenario):
    #TODO: parameters pass-in, not hard-code 
    def __init__(self, ego, agents,config = None,sim=None):
        self.sim = sim
        self._npc_drive_distance = 200 
        self._npc_speed = 10
        super(NpcAbnormalSpeed, self).__init__("NpcAbnormalSpeed", ego, agents,config=config)         
        input("NpcAbnormalSpeed Scenario hold on...")


    def _initialize_actors(self,config):
        for actor in config.other_actors:
            npc_transform = self.sim.map_point_on_lane(actor.transform.position)
            #TODO: set npc names by configure or command line input
            npc = ServerActorPool.request_new_npc(actor.rolename, npc_transform)
            self.other_actors.append(npc)

        ServerDataProvider.register_actors(self.other_actors)  

        self.logger.log.debug("size of other_acters: %d" % len(self.other_actors))
        for actor in self.other_actors:
            log = actor.state.transform.position 
            self.logger.log.info("npc %s initial position (%4.2f, %4.2f, %4.2f)" % (actor.name, log.x, log.y, log.z))
            self.logger.log.info("npc %s max speed: %s " % (actor.name, MAX_NPC_SPEED))


    def _create_behavior(self):
        npc_travel_first_half =  py_trees.composites.Parallel("npc travel with abnormal speed",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        npc_navigation = FollowClosestLane(self.other_actors[0], self._npc_speed)
        npc_drive_distance = DriveDistance(self.other_actors[0], self._npc_drive_distance/40)
        npc_travel_first_half.add_child(npc_navigation)
        npc_travel_first_half.add_child(npc_drive_distance)
        npc_sudden_stop = StopVehicle(self.other_actors[0])
        npc_stand_still = StandStill(self.other_actors[0], name="stand_still", duration=5.0)
        sequence = py_trees.composites.Sequence("npc abnormal speed sequence behavior")
        sequence.add_child(npc_travel_first_half)
        sequence.add_child(npc_sudden_stop)
        sequence.add_child(npc_stand_still)

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

   
