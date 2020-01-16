import py_trees 
import math
import lgsvl 
from scenario.server_data_provider import ServerDataProvider, ServerActorPool 
import scenario.criteria 


class Criterion(py_trees.behaviour.Behaviour):
    def __init__(self, name, actor, expected_value_success):
        super(Criterion, self).__init__(name)
        self.name = name
        self.actor = actor 
        self.test_status = "INIT"
        self.expected_value_success = expected_value_success 

def cb_collision(actor1, actor2, contact):
    raise criteria.TestException("actor {} collision with actor {}".format(actor1, actor2))

class CollisionTest(Criterion):
    def __init__(self, actor, name="CheckCollisions" ):
        super(CollisionTest, self).__init__(name, actor, 0)
        self.actor.on_collision(cb_collision)
    
    def update(self):
         new_status = py_trees.common.Status.RUNNING
         if self.test_status == "FAILURE":
            new_status = py_trees.comon.Status.FAILURE 
#TODO
         return new_status 

class MaxVelocityTest(Criterion):
    def __init__(self, actor, max_velocity_allowed, name="CheckMaxVelocity"):
        super(MaxVelocityTest, self).__init__(name, actor, max_velocity_allowed)

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self.actor is None: 
            return new_status 
        
        velocity = ServerDataProvider.get_velocity(self.actor)

        if velocity > self.expected_value_success: 
            self.test_status = "FAILURE"
            new_status = py_trees.common.Status.FAILURE
        else:
            self.test_status = "SUCCESS"

        return new_status 
        
class MaxAngularVelocityTest(Criterion):
    def __init__(self, actor, max_angular_velocity_allowed, name="CheckMaxAngularVelocity"):
        super(MaxAngularVelocityTest, self).__init__(name, actor, max_angular_velocity_allowed)

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self.actor is None: 
            return new_status 
        
        angular_velocity = self.actor.state.angular_velocity.y

        if angular_velocity > self.expected_value_success: 
            self.test_status = "FAILURE"
            new_status = py_trees.common.Status.FAILURE
        else:
            self.test_status = "SUCCESS"

        print("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

def calculate_distance(location1, location2):
    distance = (location1.x-location2.x)**2 + (location1.y-location2.y)**2 + (location1.z-location2.z)**2
    return math.sqrt(distance)   

class SeparationDistanceTest(Criterion):
    def __init__(self, actor, target_actor, max_separation_distance_allowed, end_time=False, name="CheckSeparationDistance"):
        self.target = target_actor
        #TODO: this criteria only works at final time once 
        self.at_end_time = end_time  
        super(SeparationDistanceTest, self).__init__(name, actor, max_separation_distance_allowed)
   
    def update(self):        
        new_status = py_trees.common.Status.RUNNING
        
        if self.actor is None or self.target is None:
            return new_status 

        actor_location = ServerDataProvider.get_location(self.actor)
        target_location = ServerDataProvider.get_location(self.target)

        #TODO: check direction
        if calculate_distance(target_location, actor_location) >  self.expected_value_success:
            self.test_status = "FAILURE"
            new_status = py_trees.common.Status.FAILURE 
        else:
            self.test_status = "SUCCESS"

        return new_status
    





