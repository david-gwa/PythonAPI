#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides all atomic scenario behaviors required to realize
complex, realistic scenarios such as "follow a leading vehicle", "lane change",
etc.

The atomic behaviors are implemented with py_trees.
"""

from __future__ import print_function

import random
import math
import logging
import operator
import py_trees
import lgsvl
import numpy as np
from py_trees.blackboard import Blackboard
from scenario.server_data_provider import ServerDataProvider, calculate_velocity
from scenario.timer import GameTime
from lgsvl.geometry import Vector, Transform, BoundingBox

EPSILON = 0.001


def calculate_distance(a, b):
    """
    Method to calculate the distance between to locations

    Note: It uses the direct distance between the current location and the
          target location to estimate the time to arrival.
          To be accurate, it would have to use the distance along the
          (shortest) route between the two locations.
    """
    if(a == None or b == None):
        return 0.0
    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
 
def get_actor_control(actor):
    """
    Method to return the type of control to the ego actor.
    """
    control = None
    if isinstance(actor, lgsvl.EgoVehicle): 
        control = lgsvl.VehicleControl()
        control.steering = 0.0
        control.throttle = 0.0
        control.brake = 0.0     
    return control

def normalized_vector(vec):
    s =  math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)
    if s > 1e-2:
        return lgsvl.Vector(vec.x / s,  vec.y /s ,  vec.z /s )
    return lgsvl.Vector(0.0, 0.0, 0.0)

def multiply_vector(vec, scalar):
    return lgsvl.Vector(vec.x * scalar,  vec.y * scalar, vec.z * scalar)

def set_velocity(actor, speed):
    s = actor.state 
    s.velocity = lgsvl.Vector(math.sin(math.radians(s.rotation.y))*speed, 0, math.cos(math.radians(s.rotation.y))*speed)
    actor.state = s 

class AtomicBehavior(py_trees.behaviour.Behaviour):

    """
    Base class for all atomic behaviors used to setup a scenario

    Important parameters:
    - name: Name of the atomic behavior
    """

    def __init__(self, name):
        super(AtomicBehavior, self).__init__(name)
        self.logger = logging
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.name = name
        print("%s.__init__()" % (self.__class__.__name__))

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class StandStill(AtomicBehavior):

    """
    This class contains a standstill behavior of a scenario
    """

    def __init__(self, actor, name="stand still", duration=float("inf")):
        """
        Setup actor
        """
        super(StandStill, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor

        self._duration = duration
        self._start_time = 0

    def initialise(self):
        self._start_time = GameTime.get_time()
        super(StandStill, self).initialise()

    def update(self):
        """
        Check if the _actor stands still (v=0)
        """
        new_status = py_trees.common.Status.RUNNING

        #TODO: agent's velocity doesn't keep still, the following line is a temp solution
        set_velocity(self._actor, 0.0)  
        if GameTime.get_time() - self._start_time > self._duration:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InTriggerRegion(AtomicBehavior):

    """
    This class contains the trigger region (condition) of a scenario
    """

    def __init__(self, actor, min_x, max_x, min_y, max_y, name="TriggerRegion"):
        """
        Setup trigger region (rectangle provided by
        [min_x,min_y] and [max_x,max_y]
        """
        super(InTriggerRegion, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._min_x = min_x
        self._max_x = max_x
        self._min_y = min_y
        self._max_y = max_y

    def update(self):
        """
        Check if the _actor location is within trigger region
        """
        new_status = py_trees.common.Status.RUNNING

        location = ServerDataProvider.get_location(self._actor)

        if location is None:
            return new_status

        not_in_region = (location.x < self._min_x or location.x > self._max_x) or (
            location.y < self._min_y or location.y > self._max_y)
        if not not_in_region:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InTriggerDistanceToVehicle(AtomicBehavior):

    """
    This class contains the trigger distance (condition) between to actors
    of a scenario
    """

    def __init__(self, other_actor, actor, distance,comparison_operator=operator.lt,
                 name="TriggerDistanceToVehicle"):
        """
        Setup trigger distance
        """
        super(InTriggerDistanceToVehicle, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._other_actor = other_actor
        self._actor = actor
        self._distance = distance

    def update(self):
        """
        Check if the ego vehicle is within trigger distance to other actor
        """
        new_status = py_trees.common.Status.RUNNING

        ego_location = ServerDataProvider.get_location(self._actor)
        other_location = ServerDataProvider.get_location(self._other_actor)
        new_status = py_trees.common.Status.SUCCESS

        if ego_location is None or other_location is None:
            return new_status

        if calculate_distance(ego_location, other_location) < self._distance:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InTriggerDistanceToLocation(AtomicBehavior):

    """
    This class contains the trigger (condition) for a distance to a fixed
    location of a scenario
    """

    def __init__(self, actor, target_location, distance, name="InTriggerDistanceToLocation"):
        """
        Setup trigger distance
        """
        super(InTriggerDistanceToLocation, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target_location = target_location
        self._actor = actor
        self._distance = distance

    def update(self):
        """
        Check if the actor is within trigger distance to the target location
        """
        new_status = py_trees.common.Status.RUNNING

        location = ServerDataProvider.get_location(self._actor)

        if location is None:
            return new_status

        if calculate_distance(
                location, self._target_location) < self._distance:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status


class InTimeToArrivalToLocation(AtomicBehavior):

    """
    This class contains a check if a actor arrives within a given time
    at a given location.
    """

    _max_time_to_arrival = float('inf')  # time to arrival in seconds

    def __init__(self, actor, time, location, name="TimeToArrival"):
        """
        Setup parameters
        """
        super(InTimeToArrivalToLocation, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._time = time
        self._target_location = location

    def update(self):
        """
        Check if the actor can arrive at target_location within time
        """
        new_status = py_trees.common.Status.RUNNING

        current_location = ServerDataProvider.get_location(self._actor)

        if current_location is None:
            return new_status

        distance = calculate_distance(current_location, self._target_location)
        velocity = ServerDataProvider.get_velocity(self._actor)

        # if velocity is too small, simply use a large time to arrival
        time_to_arrival = self._max_time_to_arrival
        if velocity > EPSILON:
            time_to_arrival = distance / velocity

        if time_to_arrival < self._time:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

class InTimeToArrivalToVehicle(AtomicBehavior):

    """
    This class contains a check if a actor arrives within a given time
    at another actor.

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - name: Name of the condition
    - time: The behavior is successful, if TTA is less than _time_ in seconds
    - other_actor: Reference actor used in this behavior

    The condition terminates with SUCCESS, when the actor can reach the other vehicle within the given time
    """

    _max_time_to_arrival = float('inf')  # time to arrival in seconds

    def __init__(self, other_actor, actor, time, comparison_operator=operator.lt, name="TimeToArrival"):
        """
        Setup parameters
        """
        super(InTimeToArrivalToVehicle, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._other_actor = other_actor
        self._actor = actor
        self._time = time
        self._comparison_operator = comparison_operator

    def update(self):
        """
        Check if the ego vehicle can arrive at other actor within time
        """

        new_status = py_trees.common.Status.RUNNING

        current_location=Vector()
        target_location=Vector()
        current_velocity=0.0
        other_velocity=0.0

        if self._actor.agent_type == lgsvl.AgentType.EGO:
            ego_state = self._actor.state
            current_location = ego_state.transform.position
            current_velocity = calculate_velocity(ego_state.velocity)
        else:
            current_location = ServerDataProvider.get_location(self._actor)
            current_velocity = ServerDataProvider.get_velocity(self._actor)
        
        if self._other_actor.agent_type == lgsvl.AgentType.EGO:
            ego_state = self._other_actor.state
            target_location = ego_state.transform.position
            other_velocity =  calculate_velocity(ego_state.velocity)
        else:
            target_location = ServerDataProvider.get_location(self._other_actor)
            other_velocity = ServerDataProvider.get_velocity(self._other_actor)

        if current_location is None or target_location is None:
            return new_status

        distance = calculate_distance(current_location, target_location)

        # if velocity is too small, simply use a large time to arrival
        time_to_arrival = self._max_time_to_arrival

        if current_velocity > other_velocity:
            time_to_arrival = 2 * distance / (current_velocity - other_velocity)

        if self._comparison_operator(time_to_arrival, self._time):
            new_status = py_trees.common.Status.SUCCESS
        print("dist %f cur vel %f other vel %f ttc %f " % (distance,current_velocity,other_velocity,time_to_arrival))

        return new_status

class AfterTerminationCondition(AtomicBehavior):

    """
    This class contains a check if a named story element has terminated.

    Important parameters:
    - element_name: The story element's name attribute
    - element_type: The element type [act,scene,maneuver,event,action]

    The condition terminates with SUCCESS, when the named story element ends
    """

    def __init__(self, element_type, element_name, rule):
        """
        Setup element details
        """
        super(AfterTerminationCondition, self).__init__("AfterTerminationCondition")
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._element_type = element_type.upper()
        self._element_name = element_name
        self._rule = rule.upper()
        self._start_time = GameTime.get_time()
        self._blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        """
        Check if the specified story element has ended since the beginning of the condition
        """
        new_status = py_trees.common.Status.RUNNING
        if self._rule == "ANY":
            rules = ["END", "CANCEL"]
        else:
            rules = [self._rule]

        for rule in rules:
            if new_status == py_trees.common.Status.RUNNING:
                blackboard_variable_name = "({}){}-{}".format(self._element_type, self._element_name, rule)
                element_start_time = self._blackboard.get(blackboard_variable_name)
                if element_start_time and element_start_time >= self._start_time:
                    new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

class AccelerateToVelocity(AtomicBehavior):

    """
    This class contains an atomic acceleration behavior. The controlled
    traffic participant will accelerate with _throttle_value_ until reaching
    a given _target_velocity_

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - target ax: The amount of target ax used to accelerate, -100 means stop acceleration
       and return to normal speed control
    - target_velocity: The target velocity the actor should reach in m/s

    The behavior will terminate, if the actor's velocity is at least target_velocity
    """

    def __init__(self, actor, target_ax, target_velocity, name="Acceleration"):
        """
        Setup parameters including acceleration value (via throttle_value)
        and target velocity
        """
        super(AccelerateToVelocity, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._control = lgsvl.NPCControl()
        self._actor = actor
        self.target_ax = target_ax
        self._target_velocity = target_velocity
        self._control.target_speed = self._target_velocity

    def initialise(self):
        super(AccelerateToVelocity, self).initialise()

    def update(self):
        """
        Set npc  to target npc, as long as velocity is < target_velocity
        """
        new_status = py_trees.common.Status.RUNNING
        print("%f %f %f " % (ServerDataProvider.get_velocity(self._actor),self.target_ax,self._target_velocity))
        if ServerDataProvider.get_velocity(self._actor) < self._target_velocity:
            self._control.external_acceleration = self.target_ax
        else:
            new_status = py_trees.common.Status.SUCCESS
            self._control.external_acceleration = -100
            
        self._actor.apply_control(self._control)

        return new_status


class SetToVelocity(AtomicBehavior):

    """
    This class contains an atomic acceleration behavior. The controlled
    traffic participant will accelerate with _throttle_value_ until reaching
    a given _target_velocity_  by a delay_time
    """

    def __init__(self, actor, target_velocity, delay_time=0.0, name="Acceleration"):
        """
        Setup parameters including acceleration value (via throttle_value)
        and target velocity
        """
        super(SetToVelocity, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._target_velocity = target_velocity
        self._delay_time = delay_time 

    def initialise(self):
        self._start_time = GameTime.get_time()
        super(SetToVelocity, self).initialise()
    
    def update(self):
        """
        Set throttle to throttle_value, as long as velocity is < target_velocity
        """
        new_status = py_trees.common.Status.RUNNING
        print("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        if GameTime.get_time() - self._start_time > self._delay_time:        
            set_velocity(self._actor, self._target_velocity)
            #TODO, update velocity won't get registered in ServrDataProvider ...
            new_status = py_trees.common.Status.SUCCESS
                
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

class KeepVelocity(AtomicBehavior):

    """
    This class contains an atomic behavior to keep the provided velocity.
    The controlled traffic participant will accelerate as fast as possible
    until reaching a given _target_velocity_, which is then maintained for
    as long as this behavior is active.

    Important parameters:
    - actor: CARLA actor to execute the behavior
    - target_velocity: The target velocity the actor should reach
    - duration[optional]: Duration in seconds of this behavior
    - distance[optional]: Maximum distance in meters covered by the actor during this behavior

    A termination can be enforced by providing distance or duration values.
    Alternatively, a parallel termination behavior has to be used.
    """

    def __init__(self, actor, target_velocity, duration=float("inf"), distance=float("inf"), name="KeepVelocity"):
        """
        Setup parameters including acceleration value (via throttle_value)
        and target velocity
        """
        super(KeepVelocity, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._actor = actor
        self._target_velocity = target_velocity

        self._control= get_actor_control(actor)
        self._type='vehicle' # TODO just for debug, need further analysis

        self._duration = duration
        self._target_distance = distance
        self._distance = 0
        self._start_time = 0
        self._location = None

    def initialise(self):
        self._location = ServerDataProvider.get_location(self._actor)
        self._start_time = GameTime.get_time()

        # In case of walkers, we have to extract the current heading
        if self._type == 'walker':
            self._control.speed = self._target_velocity
            self._control.direction = ServerDataProvider.get_transform(self._actor).get_forward_vector()

        super(KeepVelocity, self).initialise()

    def update(self):
        """
        As long as the stop condition (duration or distance) is not violated, set a new vehicle control

        For vehicles: set throttle to throttle_value, as long as velocity is < target_velocity
        For walkers: simply apply the given self._control
        """
        new_status = py_trees.common.Status.RUNNING

        if self._type == 'vehicle':
            if ServerDataProvider.get_velocity(self._actor) < self._target_velocity:
                self._control.throttle = 1.0
            else:
                self._control.throttle = 0.0
        self._actor.apply_control(self._control)

        new_location = ServerDataProvider.get_location(self._actor)
        self._distance += calculate_distance(self._location, new_location)
        self._location = new_location

        if self._distance > self._target_distance:
            new_status = py_trees.common.Status.SUCCESS

        if GameTime.get_time() - self._start_time > self._duration:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        return new_status

    def terminate(self, new_status):
        """
        On termination of this behavior, the throttle should be set back to 0.,
        to avoid further acceleration.
        """

        if self._type == 'vehicle':
            self._control.throttle = 0.0
        elif self._type == 'walker':
            self._control.speed = 0.0
        if self._actor is not None and self._actor.is_alive:
            self._actor.apply_control(self._control)
        super(KeepVelocity, self).terminate(new_status)


class DriveDistance(AtomicBehavior):

    """
    This class contains an atomic behavior to drive a certain distance.
    currently can not get ego car poz via ServerDataProvider.get_location(self._actor)
    so this class can not work yet
    """

    def __init__(self, actor, distance, name="DriveDistance"):
        """
        Setup parameters
        """
        super(DriveDistance, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._target_distance = distance
        self._distance = 0
        self._location = None
        self._actor = actor

    def initialise(self):
        self._location = self._actor.state.transform.position
        super(DriveDistance, self).initialise()

    def update(self):
        """
        Check driven distance
        """
        new_status = py_trees.common.Status.RUNNING
        new_location = self._actor.state.transform.position # ServerDataProvider.get_location(self._actor)  #new_location is lgsvl.Vector()
        self._distance += calculate_distance(self._location, new_location)
        self._location = new_location

        if self._distance > self._target_distance:
            new_status = py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


class StopVehicle(AtomicBehavior):

    """
    This class contains an atomic stopping behavior. The controlled traffic
    participant will decelerate with _bake_value_ until reaching a full stop.
    """

    def __init__(self, actor, brake_value=0.0, name="Stopping"):
        """
        Setup _actor and maximum braking value
        """
        super(StopVehicle, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self._control = get_actor_control(actor)
        self._actor = actor
        self._brake_value = brake_value

    def update(self):
        """
        Set brake to brake_value until reaching full stop
        """
        new_status = py_trees.common.Status.RUNNING

        if isinstance(self._actor, lgsvl.NpcVehicle) :
            if ServerDataProvider.get_velocity(self._actor) > EPSILON:
                set_velocity(self._actor, 0.0)
            else:
                new_status = py_trees.common.Status.SUCCESS
        elif isinstance(self._actor, lgsvl.EgoVehicle): 
            if ServerDataProvider.get_velocity(self._actor) > EPSILON:
                self._control.brake = self._brake_value 
            else:
                new_status = py_trees.common.Status.SUCCESS    
                self._control.brake = 0.0 
            if isinstance(self._actor, lgsvl.AgentType.EGO):
                self._actor.apply_control(self._control)
        else:
            new_status = py_trees.common.Status.SUCCESS
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


class Idle(AtomicBehavior):

    """
    This class contains an idle behavior scenario
    """

    def __init__(self, name="Idle"):
        """
        Setup actor
        """
        super(Idle, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        return new_status


class FollowClosestLane(AtomicBehavior):
    """
    This is an atomic behavior to follow closest lane to npc at the
    time when this function is executed while maintaining a given max speed,
    
    target_speed: max npc speed, unit km/h
    """
    def __init__(self, actor, target_speed, name="FollowClosestLane"):
        super(FollowClosestLane, self).__init__(name)
        self._actor = actor 
        self._target_speed = target_speed 
        self._actor.follow_closest_lane(True, self._target_speed, False) 

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        return new_status       

class FollowLaneOffset(AtomicBehavior):
    pass 

class WaypointFollower(AtomicBehavior):

    """
    This is an atomic behavior to follow waypoints indefinitely
    while maintaining a given speed or if given a waypoint plan,
    follows the given plan
    in lg, using  follow_closest_lane()
    """

    def __init__(self, actor, target_speed, blackboard_queue_name=None,
                 avoid_collision=False, name="FollowWaypoints"):
        """
        Set up actor and local planner
        """
        super(WaypointFollower, self).__init__(name)
        self._actor_list = []
        self._actor = actor
        self._actor_list.append(actor)
        self._target_speed = target_speed
        self._local_planner_list = []
        self._blackboard_queue_name = blackboard_queue_name
        if blackboard_queue_name is not None:
            self._queue = Blackboard().get(blackboard_queue_name)
        self._args_lateral_dict = {'K_P': 1.0, 'K_D': 0.01, 'K_I': 0.0, 'dt': 0.05}
        self._avoid_collision = avoid_collision

    def update(self):
        """
        Run local planner, obtain and apply control to actor
        """

        new_status = py_trees.common.Status.RUNNING

        if self._blackboard_queue_name is not None:
            while not self._queue.empty():
                actor = self._queue.get()
                if actor is not None and actor not in self._actor_list:
                    self._actor_list.append(actor)
        return new_status

    def terminate(self, new_status):
        """
        On termination of this behavior,
        the throttle, brake and steer should be set back to 0.
        """
        control = lgsvl.VehicleControl()
        control.throttle = 0.0
        control.brake = 0.0
        control.steer = 0.0
        for actor, local_planner in zip(self._actor_list, self._local_planner_list):
            if actor is not None:
                actor.apply_control(control)
            if local_planner is not None:
                local_planner.reset_vehicle()
                local_planner = None
        super(WaypointFollower, self).terminate(new_status)


class ActorTransformSetter(AtomicBehavior):

    """
    This class contains an atomic behavior to set the transform
    of an actor.
    """

    def __init__(self, actor, transform, name="ActorTransformSetter"):
        """
        Init
        """
        super(ActorTransformSetter, self).__init__(name)
        self._actor = actor
        self._transform = transform
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self):
        """
        Transform actor
        """
        new_status = py_trees.common.Status.RUNNING
        if self._actor:
            s = self._actor.state 
            s.velocity = lgsvl.Vector(0,0,0)
            s.angular_velocity = lgsvl.Vector(0, 0, 0)
            if(self._transform):
                s.transform = self._transform 
            self._actor.state = s 
            new_status = py_trees.common.Status.SUCCESS
        else:
            # For some reason the actor is gone...
            new_status = py_trees.common.Status.FAILURE
        return new_status

class NpcChangeLane(AtomicBehavior):

    def __init__(self, actor, isLeftChange=False, name="NpcChangeLane"):
        super(NpcChangeLane, self).__init__(name)
        self._actor = actor 
        self.lane_changed = False 
        self.isLeftChange = isLeftChange 
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def on_lane_changing(self, actor):
        pass 

    def on_lane_changed(self, agent, finished):
        self.lane_changed = finished 

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if not self.lane_changed:
            self._actor.on_lane_change(self.on_lane_changing) 
            set_velocity(self._actor, 10)
            self._actor.change_lane(self.isLeftChange) 
            self._actor.on_lane_change_done(self.on_lane_changed)
            return new_status         
        else: 
            return py_trees.common.Status.SUCCESS
            
class PedestrainWalkTowardVehicle(AtomicBehavior):

    """
        this behavior defines pedestrain walks toward ego vehicle 
    """

    def __init__(self, actor, target_actor, speed, name="PedestrainWalkTowardVehicle"):
        super(PedestrainWalkTowardVehicle, self).__init__(name)
        self._actor = actor 
        self._target_actor = target_actor 
        self._speed = speed 
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        dis_target = self._target_actor.state.transform.position - self._actor.state.transform.position
        walk_direction = normalized_vector(dis_target)
        set_velocity(self._actor, walk_direction * self._speed)
        if dis_target < 0.5 :
            return py_trees.common.Status.SUCCESS
        
class PedestrainWalkRandomly(AtomicBehavior):

    """
        this behavior defines pedestrain walks randomly
    """

    def __init__(self, actor, duration, name="PedestrainWalkRandomly"):
        super(PedestrainWalkRandomly, self).__init__(name)
        self._ped = actor 
        self._duration = duration
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        self._start_time = GameTime.get_time()
        super(PedestrainWalkRandomly, self).initialise()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        self._ped.walk_randomly(True)
        if GameTime.get_time() - self._start_time > self._duration:
            new_status = py_trees.common.Status.SUCCESS
        return new_status 

class PedestrainStandStill(AtomicBehavior):

    """
        this behavior defines pedestrain stand still
    """

    def __init__(self, actor, duration=10.0, name="PedestrainStandStill"):
        super(PedestrainStandStill, self).__init__(name)
        self._ped = actor 
        self._duration = duration
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        self._start_time = GameTime.get_time()
        super(PedestrainStandStill, self).initialise()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if GameTime.get_time() - self._start_time > self._duration:
            new_status = py_trees.common.Status.SUCCESS
        return new_status 