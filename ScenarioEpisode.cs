using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using System.Linq ; 
using SimpleJSON;
using UnityEngine;        
        
        class  ScenarioEpisode : System.Object 
        {                    
            public VehicleController egoManager ;
            public JSONObject EpisodeState ; 
            public List<GameObject> agents_go ; 
            public GameObject ego_go ; 

            private List<string> active_npcs ; 
          
            public void start()
            {
                EpisodeState = new JSONObject(); 
                EpisodeState.Add("type", "episode"); 
                ego_go = null ;
                agents_go = new List<GameObject>();
                egoManager = null ;
                active_npcs = new List<string>();
            }


            // return false, means agents list no update  
            private void check_active_npcs(Dictionary<string, GameObject> agents)
            {
                List<string> agent_keys = new List<string>(); 
                foreach(var agent in agents){
                    agent_keys.Add(agent.Key);
                }

                if(! active_npcs.SequenceEqual(agent_keys))
                {
                    agents_go.Clear() ; 
                    active_npcs = new List<string>(agent_keys) ;                   
                    foreach(var agent in agents)
                    { 
                        agents_go.Add(agent.Value);
                    }
                }
            }
              
            public void update()
            {     
                if(ApiManager.Instance == null)
                {
                    UnityEngine.Debug.Log("wait for ApiManager load in"); 
                    Thread.Sleep(200);
                }
                if(ego_go == null)
                {
                    ego_go = GameObject.FindGameObjectWithTag("Player"); 
                    if(egoManager == null)
                    {
                    //  egoManager =  ego_go.GetComponent<VehicleController>();  
                    }
                }

                check_active_npcs(ApiManager.Instance.Agents) ;                            
                var ego_state = new JSONObject();
                var npcs_state = new JSONArray();
                var ego_tr = ego_go.transform ; 
                var rb = ego_go.GetComponent<Rigidbody>(); 
                var _transform = new JSONObject();
                _transform.Add("position", ego_tr.position);
                _transform.Add("rotation", ego_tr.rotation.eulerAngles);
                var ego_uid = ApiManager.Instance.Agents.FirstOrDefault(item => item.Value == ego_go).Key ;
                if(ego_uid != null)
                    ego_state.Add("agent_id", ego_uid); 
                ego_state.Add("transform", _transform);
                ego_state.Add("velocity", rb.velocity);
                ego_state.Add("angular_velocity", rb.angularVelocity);
                List<GameObject> npcs_go = new List<GameObject>();
                npcs_go = agents_go.Skip(1).ToList(); 
                foreach(var npc in npcs_go)
                {
                    if(!npc) return ; 
                    //TODO: GO has been destroyed but you are still trying to access 
                    var tr =  npc.transform ;
                    var npc_transform = new JSONObject();
                    npc_transform.Add("position", tr.position);
                    npc_transform.Add("rotation", tr.rotation.eulerAngles);
                    var state = new JSONObject();
                    var npc_uid = ApiManager.Instance.Agents.FirstOrDefault(item => item.Value == npc).Key ;
                    if(npc_uid != null)
                        state.Add("agent_id", npc_uid);   
                    else 
                        state.Add("agent_id", null);                  
                    state.Add("transform", npc_transform);
                    /* 
                    TODO: with simplePhysics, it returns (0, 0,0)
                    */
                    var npcccontroller = npc.GetComponent<NPCControllerComponent>();
                    if(npcccontroller == null){
                        UnityEngine.Debug.Log("can't get npc controller");  
                        continue;  //TODO, unactive npc
                    }else{
                        var npc_rb = npc.GetComponent<Rigidbody>();
                        if(npc_rb.velocity.x < 1e-4 && npc_rb.velocity.y < 1e-4 && npc_rb.velocity.z < 1e-4 )
                        {
                            state.Add("velocity", npc_rb.velocity);
                            state.Add("angular_velocity", npcccontroller.GetAngularVelocity());
                        }else{ 
                            state.Add("velocity", npcccontroller.GetVelocity());
                            state.Add("angular_velocity", npcccontroller.GetAngularVelocity());
                        }
                    }
                    npcs_state.Add(state); 
                }
            //    EpisodeState.Add("ego_state", ego_state);
                EpisodeState.Add("npcs_state", npcs_state);
                var game_time = new JSONObject();
                game_time.Add("current_time",  ApiManager.Instance.CurrentTime);
                game_time.Add("current_frame",  ApiManager.Instance.CurrentFrame);
                EpisodeState.Add("game_time", game_time);
            }
            
        }