#!/usr/bin/env python3

import os
import lgsvl
import sys
import time
import argparse
from argparse import RawTextHelpFormatter
import scenario.criteria
from scenario.logger import *
from scenario.scenario_manager import ScenarioManager 
from scenario.actor_pos import ActorPos
from scenario.world import World
from scenario.open_scenario import OpenScenario
from scenarioconfigs.openscenario_configuration import OpenScenarioConfiguration
from scenario.server_data_provider import * 
from scenario.follow_leading_vehicle import *
from scenario.npc_cut_in import *
from scenario.npc_cut_off import * 
from scenario.obstacle_in_front import *
from scenario.npc_abnormal_speed import *
from scenario.npc_speed_profile import *
from scenario.pedestrain_stay_in_front import * 
from scenario.npc_traffic_jam import * 
from scenario.npc_ttc_trigger import *

def checkPositionNull(position):
    if position.x == 0.0 and position.y == 0.0 and position.z == 0.0:
        return True
    return False

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
SPEED_VARIANCE = 4 # Without real physics, the calculation for a rigidbody's velocity is very imprecise
MAX_POV_SPEED = 8.94 # (32 km/h, 20 mph)
MAX_POV_ROTATION = 5 #deg/s
TIME_LIMIT = 45 # seconds
TIME_DELAY = 3 # The EGO starts moving before the POV to allow it to catch up
MAX_FOLLOWING_DISTANCE = 10 # The maximum distance the EGO should be from the POV 

SCENARIOS = {"FollowLeadingVehicle" : FOLLOW_LEADING_VEHICLE_SCENARIO , 
             "NpcCutOff": NPC_CUT_OFF_SCENARIO ,
             "NpcCutIn" : NPC_CUT_IN_SCENARIO , 
             "ObstacleInFront":  OBSTACLE_IN_FRONT_SCENARIO,
             "NpcAbnormalSpeed": NPC_ABNORMAL_SPEED_SCENARIO,
             "NpcSpeedProfile": NPCSPEEDPROFILE ,
             "PedestrainStillInFront" : PEDESTRAIN_STILL_IN_FRONT,
             "NpcTrafficJam": NPC_TRAFFIC_JAM,
             "NpcTTCTrigger": NPC_TTC_TRIGGER_SCENARIO,
            }

class ScenarioRunner(object):
    def __init__(self, simulatorInstance, args):
        self.ego = None 
        self.agents = []
        self.manager = None 
        self.logger = logger(__name__)
        self.logger.set_output_file()
        self.sim = simulatorInstance 
        self.scenario_name = args.scenario  #not instance
        self.run_mode = args.run_mode 
        self.world = None
        self.ego_vehicles = []
        if self.run_mode == "stand_alone":
            self.repetitions = 1
        else:
            self.repetitions = 10 #to test cloud run 
        self.config_dic = {}

    def lookup_scenario(self, scenario):
        for scenarios in SCENARIOS.values():
            if scenario in scenarios:
                if scenario in globals():
                    return globals()[scenario]

    
    def prepare_ego(self, ego_position=None,config=None):
        egoState = lgsvl.AgentState()
        if ego_position is not None:
            egoState.transform = self.sim.map_point_on_lane(ego_position)
            _position = egoState.transform.position
            _position.z -= 3.5
            egoState.transform.position = _position
        elif config.ego_vehicles is not None: # config info in openscenario file
            tf=config.ego_vehicles[0].transform
            egoState.transform = self.sim.map_point_on_lane(tf.position)            
        else: #customized ego pos input 
            pos, rot = ActorPos.init_ego_pos(True,self.sim)
            egoState.transform = self.sim.map_point_on_lane(pos)

        if checkPositionNull(egoState.transform.position) and checkPositionNull(egoState.transform.rotation):
            self.logger.log.warn("egostate transform null, ego return")
            self.ego = None 
            return 
        if self.run_mode == "stand_alone":
            self.ego = self.sim.add_agent("GWM_Ego_Vehicle_Win", lgsvl.AgentType.EGO, egoState)    
        else:        
            self.ego = self.sim.add_agent("XE_Rigged-apollo_3_5", lgsvl.AgentType.EGO, egoState)

        self.ego_vehicles.append(self.ego)

        loginfo = egoState.transform.position 
        logrotation = egoState.transform.rotation
        self.logger.log.info("ego initial position: (%4.2f, %4.2f, %4.2f)" %(loginfo.x, loginfo.y, loginfo.z))
        self.logger.log.info("ego initial rotation: (%4.2f, %4.2f, %4.2f)" %(logrotation.x, logrotation.y, logrotation.z))
        self.logger.log.debug("prepare_ego() done")
        #TODO: self.ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)

    def prepare_npc(self,config=None):
        if config is not None and config.other_actors is not None:
            npc_transform =config.other_actors[0].transform            
            npc_transform = self.sim.map_point_on_lane(npc_transform.position)
        else:
            npc_pos, npc_rotation = ActorPos.init_npc_pos(True,self.sim)
            npc_transform = self.sim.map_point_on_lane(npc_pos)

        #TODO: set npc names by configure or command line input
        npc = ServerActorPool.request_new_npc("Sedan", npc_transform)
        self.agents.append(npc)

    def load_and_run_scenario(self, scenario, sim):
        self.manager.load_scenario(scenario)
        self.logger.log.debug("manage load scenario done")
        self.manager.run_scenario(sim)
        self.logger.log.debug("manager run scenaio done")

    def cleanup(self):
        ServerDataProvider.cleanup()
        ServerActorPool.cleanup()
        self.manager.restart()
        self.sim.reset()
        self.agents = []
        self.repetitions = 1
        time.sleep(2)
        self.logger.log.debug("cleanup done")

    def _run_scenarios(self,args):
        
        config = OpenScenarioConfiguration(args.openscenario)
        
        for _ in range(self.repetitions): 
            self.logger.log.info("scenario: %s episode index: %d" % (self.scenario_name, _))
            self.manager = ScenarioManager()
            try:
                self.prepare_ego(config=config)
                if self.ego is not None: 
                    self.logger.log.debug("ego x, z position: %4.2f, %4.2f" % (self.ego.state.transform.position.x, self.ego.state.transform.position.z))     
                else :
                    self.cleanup()
                    continue 
        #        ServerDataProvider.register_actor(self.ego)
                ServerActorPool.set_world(self.sim)   
            except Exception as exception:
                self.logger.log.error("this scenario can't load successfully")
                self.logger.log.error(exception)
                return 

            scenario_class = self.lookup_scenario(self.scenario_name)
            scenario = scenario_class(self.ego, self.agents,config=config,sim=self.sim)
            self.load_and_run_scenario(scenario, self.sim)
            self.cleanup() 

    def _run_openscenario(self,args):
        """
        Run a scenario based on OpenSCENARIO
        """
        # Load the scenario configurations provided in the config file
        if not os.path.isfile(args.openscenario):
            print("File does not exist")
            self.cleanup()
            return

        config = OpenScenarioConfiguration(args.openscenario)
        
        self.manager = ScenarioManager()
        try:
            self.prepare_ego(config=config)
            if self.ego is not None: 
                self.logger.log.debug("ego x, z position: %4.2f, %4.2f" % (self.ego.state.transform.position.x, self.ego.state.transform.position.z))     
            else :
                print("ego vehicle is not exist")
                self.cleanup()
                return  
    #        ServerDataProvider.register_actor(self.ego)
            ServerActorPool.set_world(self.sim)   
        except Exception as exception:
            self.logger.log.error("this scenario can't load successfully")
            self.logger.log.error(exception)
            # continue
            return 

        #judge if it is openscenario 
        if args.openscenario:
            scenario = OpenScenario(world=self.world,
                                    ego_vehicles=self.ego_vehicles,
                                    config=config,
                                    config_file=args.openscenario,
                                    sim=self.sim,
                                    timeout=100)
        self.load_and_run_scenario(scenario, self.sim)
        self.cleanup()

    def run(self,args):
        if self.run_mode == "cloud": 
            self._run_scenarios(args)
        elif args.scenario!= '': # need to think carefully about mode selection
            self._run_scenarios(args)
        else:#openscenario
            self._run_openscenario(args)
        
        input("press enter to continue")
        self.logger.log.info("all scenarios finished")         

def main():
    DESCRIPTION = ("LG-SIM Scenario Runner: Setup, Run and Evaluate scenarios using LG simulator\n")
    PARSER = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=RawTextHelpFormatter)  
    PARSER.add_argument('-map', default="LG_map", help="default map: LG_map)")    
    PARSER.add_argument('-scenario', default="NpcSpeedProfile", help="default scenario: FollowLeadingVehicle") 
    PARSER.add_argument('-openscenario', default=r'.\testcases\NpcCutIn.xosc', help='Provide an OpenSCENARIO definition')
    PARSER.add_argument('-run_mode', default="stand_alone", help="scenario run in stand_alone mode") 
    PARSER.add_argument('-time_of_day', default=10, help="time of day in current world" )    
    
    ARGUMENTS =PARSER.parse_args()
    world = World(ARGUMENTS.map)
    world.load() 
    world.set_time_of_day(ARGUMENTS.time_of_day)
    scenarioRunner = ScenarioRunner(world.sim, ARGUMENTS) 
    try:
        scenarioRunner.run(ARGUMENTS)
    except KeyboardInterrupt:
        print("\n Cancelled by user, bye")
    except Exception as error:
        print(error)

if __name__ == '__main__':
    main()