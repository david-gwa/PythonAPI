import logging
import math 
import random 
import lgsvl 


def calculate_velocity(velocity):
    if isinstance(velocity, lgsvl.Vector):
        velocity_squared = velocity.x**2
        velocity_squared += velocity.y**2
        velocity_squared += velocity.z**2 
        return math.sqrt(velocity_squared)   

def json2vector(jdic):
    assert(type(jdic) is dict)
    vec = lgsvl.Vector(0, 0, 0)
    vec.x = jdic["x"]
    vec.y = jdic["y"]
    vec.z = jdic["z"]
    return vec

class GameTime(object):
    def __init__(self):
        self.currentTime = 0.0
        self.currentFrame = 0
    
    def from_json(self, jdic):
        self.currentTime = jdic["current_time"]
        self.currentFrame = jdic["current_frame"]


class ServerDataProvider(object):
    
    _actor_velocity_map = dict() 
    _actor_location_map = dict()
    id2actor = dict()  
    game_timer = GameTime()

    @staticmethod
    def register_actor(actor):
        """
        Add new actor to dictionaries
        If actor already exists, throw an exception
        """
        if actor not in (ServerDataProvider.id2actor).values():
            id =  len(ServerDataProvider.id2actor)
            ServerDataProvider.id2actor[id] = actor 
  
        if actor in ServerDataProvider._actor_velocity_map:
            raise KeyError(
                "Vehicle '{}' already registered. Cannot register twice!".format(actor))
        else:
            ServerDataProvider._actor_velocity_map[actor] = 0.0

        if actor in ServerDataProvider._actor_location_map:
            raise KeyError(
                "Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            ServerDataProvider._actor_location_map[actor] = None

    @staticmethod
    def register_actors(actors):
        """
        Add new set of actors to dictionaries
        """
        if actors is None or len(actors)==0:
            logging.warning("register_actors with null")
            return 

        for actor in actors:
            ServerDataProvider.register_actor(actor)

    #TODO: add condition variable / lock
    @staticmethod 
    def on_server_tick(episode_state=None):  
        npcs_state = []
        id = 0  
        tmp_position = lgsvl.Vector(0,0,0)
        tmp_velocity = lgsvl.Vector(0,0,0)
        if episode_state is not None:
            npcs_state = episode_state["npcs_state"] #list
            #TODO
            for id in range(len(npcs_state)):
                cur_velocity = npcs_state[id]["velocity"]
                tmp_velocity = json2vector(cur_velocity)
                cur_position = npcs_state[id]["transform"]["position"]
                tmp_position = json2vector(cur_position)
                logging.debug("on_server_tick() tmp_velocity %4.2f, %4.2f, %4.2f" %(tmp_velocity.x, tmp_velocity.y, tmp_velocity.z))
                if id in ServerDataProvider.id2actor:
                    actor = ServerDataProvider.id2actor[id]
                    ServerDataProvider._actor_velocity_map[actor] = calculate_velocity(tmp_velocity)  
                    ServerDataProvider._actor_location_map[actor] = tmp_position           
                    logging.debug("test with id %4d, on_server_tick velocity %4.2f" % (id, ServerDataProvider.get_velocity(actor)))
                    logging.debug("test with id %4d, on_server_tick position.x %4.2f" % (id, ServerDataProvider.get_location(actor).x))                    
                else:
                    logging.debug("debug id2actor %d" % id)
            ServerDataProvider.game_timer.from_json(episode_state["game_time"])
           
    @staticmethod 
    def get_velocity(actor):
        if actor not in ServerDataProvider._actor_velocity_map.keys():
            print(actor.name+" is not in actor list")
            return 0.0
        return ServerDataProvider._actor_velocity_map[actor]

    @staticmethod
    def get_location(actor):
        if actor not in ServerDataProvider._actor_location_map.keys():
            return None
        return ServerDataProvider._actor_location_map[actor]

    @staticmethod 
    def cleanup():
        ServerDataProvider.id2actor.clear()
        ServerDataProvider._actor_velocity_map.clear()
        ServerDataProvider._actor_location_map.clear()

class ServerActorPool(object):

    _actor_pool = []
    _spawn_points = []
    _sim = None 

    @staticmethod 
    def set_world(simInstance):
        ServerActorPool._sim = simInstance 
        ServerActorPool.generate_spawn_points()

    @staticmethod
    def generate_spawn_points():
        ServerActorPool._spawn_points = ServerActorPool._sim.get_spawn()

    @staticmethod 
    def setup_actor(name, agent_type=None, spawn_point=None):
        state = lgsvl.AgentState()
        if spawn_point == None:            
            tmp_spawn_point = random.choice(ServerActorPool._spawn_points)
            state.transform = tmp_spawn_point 
        else:
            state.transform = spawn_point
        if agent_type == None:
            actor = ServerActorPool._sim.add_agent(name, lgsvl.AgentType.NPC, state)
        else:
            actor = ServerActorPool._sim.add_agent(name, agent_type, state)

        if ServerActorPool.check_agent_on_road(actor):
            logging.debug("npc position: %4.2f, %4.2f" % (actor.state.transform.position.x, actor.state.transform.position.z))
        else:
            ServerActorPool._sim.remove_agent(actor)
            return None 

        if actor is None:
            raise RuntimeError(
                "Error: Unable to spawn vehicle {} at {}".format(name, spawn_point))
        return actor

    @staticmethod 
    def request_new_npc(name, spawn_point=None):
        actor = ServerActorPool.setup_actor(name, lgsvl.AgentType.NPC, spawn_point)
        if actor is None:
            return None
        ServerActorPool._actor_pool.append(actor)
        return actor 

    @staticmethod
    def request_new_obstacle(name, spawn_point=None):
        obstacle = ServerActorPool.setup_actor(name, lgsvl.AgentType.OBSTACLE, spawn_point)
        if obstacle is None:
            return None
        ServerActorPool._actor_pool.append(obstacle)
        return obstacle 

    @staticmethod 
    def request_new_pedestrain(name, spawn_point=None):
        pedestrain = ServerActorPool.setup_actor(name, lgsvl.AgentType.PEDESTRIAN, spawn_point)
        if pedestrain is None:
            return None 
        ServerActorPool._actor_pool.append(pedestrain)
        return pedestrain 


    @staticmethod 
    def check_agent_on_road(agent, ray_direction=lgsvl.Vector(0, -1, 0)):
#TODO: layer mask doesn't work as expected 
        origin = agent.state.transform.position
        origin.y += 1 
        hit =  ServerActorPool._sim.raycast(origin, ray_direction, max_distance=10)
        if not  hit:
            print(agent.name, " is not on road, need clear\n")
            return False 
        # print(hit)
        return True 

    @staticmethod
    def cleanup():
        ServerActorPool._actor_pool.clear()
        ServerActorPool._spawn_points = []
        ServerActorPool._sim = None 

#TODO: class ServerTimer(object):
