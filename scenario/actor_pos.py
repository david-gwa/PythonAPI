import lgsvl
import math 

ego_pos_file = "scenario/ego_positions.txt"
npc_pos_file = "scenario/npcs_positions.txt"


def norm(vec):
    return  math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)

def normalized(vec):
    s = norm(vec)
    if s > 1e-2:
        return lgsvl.Vector(vec.x / s,  vec.y /s ,  vec.z /s )
    return lgsvl.Vector(0.0, 0.0, 0.0)

def dot_product(vec1, vec2):
    return  vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z 

#lgsvl rotation is in global world coordinate system 
def directionVector2Angles(final_vec, init_vec):
    ab = dot_product(final_vec, init_vec)
    final_normal = norm(final_vec)
    init_normal = norm(init_vec)
    theta = ab / final_normal / init_normal
    axis_y = math.acos(theta) / math.pi * 180
    if axis_y > 180:
        axis_y = axis_y - 360
    return lgsvl.Vector(0, axis_y, 0)
    
class ActorPos(object):

    @staticmethod
    def init_npc_pos(isGps=False, sim=None):
        npc_pos = lgsvl.Vector(0.0, 0.0, 0.0)
        npc_heading = lgsvl.Vector(0.0, 0.0, 0.0)
        npc_trans = lgsvl.Transform()
        heading_trans = lgsvl.Transform() 
        npc_fh = open(npc_pos_file, "r+")
        npc_data = npc_fh.read().splitlines()
        for _ in npc_data:
            if _.startswith("npc_pos") :
                xy = _.split()
                if isGps:
                    lon = float(xy[1])
                    lat = float(xy[2])
                    print("lat and lon: %4.6f, %4.6f" %(lat, lon))
                    npc_trans = sim.map_from_gps(latitude = lat, longitude = lon)   #TODO: restructre with sim
                else:                     
                    npc_pos.x = float(xy[1])
                    npc_pos.z = float(xy[2])
            if _.startswith("npc_heading") :
                headings = _.split()
                if isGps:
                    lon2 = float(headings[1]) 
                    lat2 = float(headings[2])
                    heading_trans = sim.map_from_gps(latitude = lat2, longitude = lon2)
                else:            
                    npc_heading.x = float(headings[1]) 
                    npc_heading.z = float(headings[2])               
        npc_fh.close()    

        if isGps:
            npc_heading = directionVector2Angles(heading_trans.position, npc_trans.position)
        else: 
            npc_heading = directionVector2Angles(npc_heading, npc_pos)

        if isGps:
            return (npc_trans.position, npc_heading)
        else: 
            return (npc_pos, npc_heading)          


    @staticmethod
    def init_ego_pos(isGps=False, sim=None): 
        ego_pos = lgsvl.Vector(0.0, 0.0, 0.0)
        ego_heading = lgsvl.Vector(0.0, 0.0, 0.0)     # the initial ego_heading is just the next point in dirving direction   
        ego_trans = lgsvl.Transform()
        heading_trans = lgsvl.Transform() 
        ego_fh = open(ego_pos_file, "r+")
        ego_data = ego_fh.read().splitlines()
        for _ in ego_data:
            if _.startswith("ego_pos"):
                xy = _.split()  
                if isGps:
                    lon = float(xy[1])
                    lat = float(xy[2])
                    ego_trans = sim.map_from_gps(latitude = lat, longitude = lon)   #TODO: restructre with sim
                else: 
                    ego_pos.x = float(xy[1])
                    ego_pos.z = float(xy[2])
            if _.startswith("ego_heading"):
                headings = _.split()  
                if isGps:
                    lon2 = float(headings[1]) 
                    lat2 = float(headings[2])
                    heading_trans = sim.map_from_gps(latitude = lat2, longitude = lon2)
                else:
                    ego_heading.x = float(headings[1]) 
                    ego_heading.z = float(headings[2])          
        ego_fh.close()
        if isGps:
            ego_heading = directionVector2Angles(heading_trans.position, ego_trans.position)
        else: 
            ego_heading = directionVector2Angles(ego_heading, ego_pos)
        print("ego heading rotation in y %4.4f" % ego_heading.y)

        if isGps:
            return (ego_trans.position, ego_heading)
        else: 
            return (ego_pos, ego_heading)          

    @staticmethod
    def npc_waypoints(self):
        pass 






        
