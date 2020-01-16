import time 
import py_trees
from scenario.logger import * 
from scenario.server_data_provider import ServerDataProvider 
from scenario.timer import GameTime, TimeOut


class Scenario():
    def __init__(self, behavior, criteria, name, timeout=30):

        self.behavior = behavior 
        self.test_criteria = criteria 
        self.timeout = timeout 

        if self.test_criteria is not None and not isinstance(self.test_criteria, py_trees.composites.Parallel):

            self.criteria_tree = py_trees.composites.Parallel(name="Test Criteria")
            self.criteria_tree.add_children(self.test_criteria)
            self.criteria_tree.setup(timeout=1)
        else:
            self.criteria_tree = criteria 
        self.timeout_node = TimeOut(self.timeout, name="TimeOut")
        self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        self.scenario_tree.add_child(self.behavior)
        self.scenario_tree.add_child(self.timeout_node)
        if self.test_criteria is not None:
            self.scenario_tree.add_child(self.criteria_tree)
        self.scenario_tree.setup(timeout=1)

    def _extract_nodes_from_tree(self, tree):  # pylint: disable=no-self-use
        """
        Returns the list of all nodes from the given tree
        """
        node_list = [tree]
        more_nodes_exist = True
        while more_nodes_exist:
            more_nodes_exist = False
            for node in node_list:
                if node.children:
                    node_list.remove(node)
                    more_nodes_exist = True
                    for child in node.children:
                        node_list.append(child)

        if len(node_list) == 1 and isinstance(node_list[0], py_trees.composites.Parallel):
            return []

        return node_list

    # will be used in result logger
    def get_criteria(self):
        """
        Return the list of test criteria (all leave nodes)
        """
        criteria_list = self._extract_nodes_from_tree(self.criteria_tree)
        return criteria_list

    #TODO: 
    def terminate(self):
        pass

class ScenarioManager(object):

    def __init__(self):
        self._running = False 
        self.scenario_tree = None 
        self.fixed_delta_time = 0.5
        self.logger = logger(__name__)
        self.logger.set_output_file() #using default log config
        self.logger.log.debug("ScenarioManager init done ")

    def load_scenario(self, scenario):
        self.restart()
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree 
        print(self.scenario_tree) 

    def restart(self):
        self._running = False 
        self.scenario = None
        self.scenario_tree = None
        GameTime.restart()

    def run_scenario(self, sim):   
        self._running = True 
        start_system_time = time.time()
        start_game_time = GameTime.get_time()
        while self._running:
            sim.run_with_cb(self.fixed_delta_time)
            if sim.episode_state is not None:
                ServerDataProvider.on_server_tick(sim.episode_state)
            self._tick_scenario()
        end_system_time = time.time()
        end_game_time = GameTime.get_time()
        self.scenario_duration_system = end_system_time - start_system_time 
        self.scenario_duration_game = end_game_time - start_game_time 
        self.logger.log.info("  scenario duration system time |     duration game time ")
        self.logger.log.info("---------------------------------------------------------")
        self.logger.log.info("%4.2f | %4.2f"%(self.scenario_duration_system, self.scenario_duration_game))

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            self.logger.log.warn("ScenarioManager: Terminated due to failure")
    
    def _tick_scenario(self):
        #TODO: return simulator currentTime and currentFrame        
        GameTime.on_server_tick(ServerDataProvider.game_timer)
        self.scenario_tree.tick_once() # generator iterate forward one step
        if self.scenario_tree.status != py_trees.common.Status.RUNNING:
            self._running = False   
            self.logger.log.info("scenario stop running with status\t %r" %  self.scenario_tree.status)
              
    def analyze_scenario(self):
        pass 
   
