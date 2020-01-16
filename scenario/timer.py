import datetime
import py_trees

class GameTime(object):

    _current_game_time = 0.0 
    _last_frame = 0 
    _platform_timestamp = 0

    @staticmethod 
    def on_server_tick(timestamp):
        if GameTime._last_frame < timestamp.currentFrame:
            GameTime._last_frame = timestamp.currentFrame 
            GameTime._current_game_time = timestamp.currentTime 

    @staticmethod 
    def restart():
        GameTime._current_game_time = 0.0
    
    @staticmethod
    def get_time():
        return GameTime._current_game_time 
    
    @staticmethod
    def get_wallclocktime():
        return GameTime._platform_timestamp


class TimeOut(py_trees.behaviour.Behaviour):
    def __init__(self, timeout, name="TimeOut"):
        super(TimeOut, self).__init__(name)
        self._timeout_value = timeout 
        self._start_time = 0.0 
        self.timeout = False 

    def initialise(self):
        self._start_time= GameTime.get_time()
    
    def update(self):
        elapsed_time = GameTime.get_time() - self._start_time 

        if elapsed_time < self._timeout_value:
            new_status = py_trees.common.Status.RUNNING
        else:
            new_status = py_trees.common.Status.SUCCESS
            self.timeout = True 
        return new_status 

class SimulationTimeCondition(py_trees.behaviour.Behaviour):

    """
    This class contains an atomic simulation time condition behavior.
    It uses the LG-SIM game time, not the system time which is used by
    the py_trees timer.

    Returns, if the provided success_rule (greater_than, less_than, equal_to)
    was successfully evaluated
    """

    def __init__(self, timeout, success_rule="greater_than", name="SimulationTimeCondition"):
        """
        Setup timeout
        """
        super(SimulationTimeCondition, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._timeout_value = timeout
        self._start_time = 0.0
        self._success_rule = success_rule
        self._ops = {"greater_than": (lambda x, y: x > y),
                     "equal_to": (lambda x, y: x == y),
                     "less_than": (lambda x, y: x < y)}

    def initialise(self):
        self._start_time = GameTime.get_time()

    def update(self):
        elapsed_time = GameTime.get_time() - self._start_time

        if not self._ops[self._success_rule](elapsed_time, self._timeout_value):
            new_status = py_trees.common.Status.RUNNING
        else:
            new_status = py_trees.common.Status.SUCCESS


        return new_status



