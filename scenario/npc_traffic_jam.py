import py_trees 
from scenario.basic_scenario import BasicScenario
from scenario.atomic_scenario_behavior import *
from scenario.atomic_scenario_criteria import *
from scenario.server_data_provider import ServerActorPool 

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
MAX_FOLLOWING_DISTANCE = 10 # The maximum distance the EGO should be from the POV 
MAX_NPC_SPEED = 10
VEHICLE_LENGTH = 6.0

NPC_TRAFFIC_JAM = ["NpcTrafficJam"]

class NpcTrafficJam(BasicScenario):
    
    def __init__(self, ego, agents,config = None,sim=None):
        self.sim = sim
        self._npc_drive_distance = 100 
        self._npc_speed = 10.0
        self._npc_stop_in_front_intersection = 20
        self._npc_max_brake = 1.0 
        self.ego = ego 
        super(NpcTrafficJam, self).__init__("NpcTrafficJam", ego, agents,config=config)  
        input("NpcTrafficJam Scenario hold on...")

    def _get_next_spawn_position(self, npc, npc2):
        npc1_trans = npc.state.transform
        npc2_trans = npc2.state.transform 
        driving_direction =  normalized_vector(npc1_trans.position - npc2_trans.position)        
        dist0 = calculate_distance(npc1_trans.position, npc2_trans.position)
        if dist0 > 10:  #TODO
            next_pos =  npc2_trans.position + dist0/2.0 * driving_direction 
        else:
            next_pos = npc1_trans.position +  5.0 * driving_direction 
        return lgsvl.Transform(next_pos, npc2_trans.rotation)

    def _get_next_npc(self, npc, layers=3):
        if layers > 0:
            if npc is not None: #TDO, npc not in other_actors
                self.other_actors.append(npc)
                prev_npc = self.other_actors[len(self.other_actors)-2]
                next_trans2 = self._get_next_spawn_position(npc, prev_npc) 
                npc22 = ServerActorPool.request_new_npc("Sedan", next_trans2)      
                self._get_next_npc(npc22, layers-1)      
        else:
            return None

# assuming the first npc and ego is in same lane 
    def _initialize_actors(self,config):
        lane_width = 4.0
        trans0 = self.other_actors[0].state.transform 
        ego_trans = self.ego.state.transform 
        driving_direction =  normalized_vector(trans0.position - ego_trans.position)
        unit_normal_direction = lgsvl.Vector(-driving_direction.z, driving_direction.y, driving_direction.x)
        print("dbug unit_normal: ", unit_normal_direction)
        trans1 = lgsvl.Transform(trans0.position,trans0.rotation) 
        trans1.position = trans0.position - lane_width * unit_normal_direction
        print("debug npc1 jam: ", trans1.position)
        npc1 = ServerActorPool.request_new_npc("Sedan", trans1)
        trans2 = lgsvl.Transform(trans0.position,trans0.rotation) 
        trans2.position = trans0.position + lane_width * unit_normal_direction 
        print("debug npc2 jam: ", trans2.position)
        npc2 = ServerActorPool.request_new_npc("Sedan", trans2)
        self._get_next_npc(npc1)
        self._get_next_npc(npc2)
        print("debug jam, npc length ", len(self.other_actors)) 

        self.logger.log.info("npc0 initial position: (%4.2f, %4.2f, %4.2f)" %(trans0.position.x, trans0.position.y, trans0.position.z))

    def _create_behavior(self):
        traffic_jam =  py_trees.composites.Parallel("traffic_jam",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        npc0_stand_still = StandStill(self.other_actors[0], name="npc0 stand_still", duration=15.0)
        npc1_stand_still = StandStill(self.other_actors[1], name="npc1 stand_still", duration=15.0)
        npc2_stand_still = StandStill(self.other_actors[2], name="npc2 stand_still", duration=15.0)
        npc_navigation =  py_trees.composites.Parallel("npc_navigation",     
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)        
        npc0_navigation = FollowClosestLane(self.other_actors[0], self._npc_speed)
        npc1_navigation = FollowClosestLane(self.other_actors[1], self._npc_speed)
        npc2_navigation = FollowClosestLane(self.other_actors[2], self._npc_speed)
        sequence = py_trees.composites.Sequence()
        traffic_jam.add_child(npc0_stand_still)
        traffic_jam.add_child(npc1_stand_still)
        traffic_jam.add_child(npc2_stand_still)
        npc_navigation.add_child(npc0_navigation)
        npc_navigation.add_child(npc1_navigation)        
        npc_navigation.add_child(npc2_navigation)
        sequence.add_child(traffic_jam)
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
   
