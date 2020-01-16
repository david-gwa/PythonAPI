#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import threading
import websockets
import asyncio
import json

class Remote(threading.Thread):

  def __init__(self, host, port):
    super().__init__(daemon=True)
    self.endpoint = "ws://{}:{}".format(host, port)
    self.lock = threading.Lock()
    self.episode_cv = threading.Condition()
    self.cv = threading.Condition() 
    self.data = None
    self.sem = threading.Semaphore(0)
    self.running = True
    self.start()
    self.sem.acquire()
    self.cv_released_already = False 
    self.episode_status = {} 

  def run(self):
    self.loop = asyncio.new_event_loop()                
    asyncio.set_event_loop(self.loop)
    self.loop.run_until_complete(self.process())

  def close(self):
    asyncio.run_coroutine_threadsafe(self.websocket.close(), self.loop)
    self.join()
    self.loop.close()

  async def process(self):
    self.websocket = await websockets.connect(self.endpoint, compression=None)
    self.sem.release()

    while True:
      try:
        data = await self.websocket.recv()
      except Exception as e:
        if isinstance(e, websockets.exceptions.ConnectionClosed):
          break
        with self.cv:
          self.data = {"error": str(e)}
          self.cv.notify()
        break   
     
      try:
        self.cv.acquire()
        self.data = json.loads(data)
        if "error" in self.data:
          break
        if type(self.data) is dict and self.data["result"] is not None and type(self.data["result"]) is dict and "type" in self.data["result"] and self.data["result"]["type"] == "episode":   
      #  if type(self.data) is dict and next(iter(self.data)) == "result" and "type" in self.data["result"] and self.data["result"]["type"] == "episode":   
          self.cv.release()
          self.cv_released_already = True  
          try:
            self.episode_cv.acquire()
            self.episode_cv.notify()
          finally:
            self.episode_status = self.episode_tick() 
            self.episode_cv.release()          
        else:
           self.cv.notify()
           self.cv_released_already = False            
      finally:
        if(self.cv_released_already == False):
          self.cv.release()

      
    await self.websocket.close()

  def command(self, name, args = {}):
    if not self.websocket:
      raise Exception("Not connected")
    data = json.dumps({"command": name, "arguments": args})
    asyncio.run_coroutine_threadsafe(self.websocket.send(data), self.loop)
    with self.cv:
      self.cv.wait_for(lambda: self.data is not None)
      data = self.data
      self.data = None
    if "error" in data:
      raise Exception(data["error"])
    return data["result"] 

  def episode_tick(self):
    if not self.websocket:
      raise Exception("Not connected")
  #  with self.episode_cv:
    self.episode_cv.acquire()
    try:
      self.episode_cv.wait_for(lambda: self.data is not None)
      data = self.data 
      self.data = None
    finally:
      self.episode_cv.release()

    if "error" in data:
      raise Exception(data["error"])
    return data["result"]
