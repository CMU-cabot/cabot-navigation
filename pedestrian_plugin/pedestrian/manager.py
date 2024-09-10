# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import logging
import uuid

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from pedestrian_plugin_msgs.msg import Agents
from pedestrian_plugin_msgs.msg import Plugin
from pedestrian_plugin_msgs.msg import PluginParam
from pedestrian_plugin_msgs.srv import PluginUpdate


def identify_variable_type(variable):
    variable_type = type(variable)
    if variable_type == int:
        return "int"
    elif variable_type == float:
        return "float"
    elif variable_type == bool:
        return "bool"
    elif variable_type == str:
        return "str"
    else:
        return "str"


class PedestrianManager():
    def __init__(self, node, callback=None):
        self.node = node
        self.spawn_entity_client = self.node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = self.node.create_client(DeleteEntity, '/delete_entity')
        self.pedestrian_plugin_update_client = self.node.create_client(PluginUpdate, '/pedestrian_plugin_update')
        self.human_states_sub = self.node.create_subscription(Agents, '/human_states', self.human_states_callback, 10)
        self.timer = self.node.create_timer(1, self.check_service)
        self.serviceReady = False
        self.actorMap = {'mobile_base': {}}
        self.futures = {}

    def human_states_callback(self, msg):
        if len(self.actorMap) < len(msg.agents):
            for agent in msg.agents:
                if agent.name not in self.actorMap:
                    self.actorMap[agent.name] = {}

    def check_service(self):
        if self.pedestrian_plugin_update_client.wait_for_service(timeout_sec=0):
            logging.debug("service available")
            self.serviceReady = True
            self.timer.cancel()

    def init(self, callback=None):
        if self.serviceReady:
            self._update(actors=[], callback=callback)
        else:
            self.update(actors=[{"name": "actor0"}], callback=callback)

    def delete(self, name=None, callback=None):
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_entity_client.call_async(request)
        self.futures[name] = future
        if callback:
            future.add_done_callback(callback)

    def update(self, actors=None, callback=None):
        if actors is None:
            logging.debug("needs to specify actors")
            return
        update_actors = []
        self.task_count = 0

        def complete(future):
            self.task_count -= 1
            logging.debug(f"remaining task = {self.task_count}")
            if self.task_count > 0:
                return

            def complete2(future):
                logging.debug(f"done complete2 {future.result()}")
                if callback:
                    callback(future)
            if len(update_actors) > 0:
                self._update(actors=update_actors, callback=complete2)
            else:
                if callback:
                    callback(future)

        if len(actors) > 0:
            alreadyAdded = {}
            for actor in actors:
                if 'name' not in actor:
                    logging.debug("needs to specify actor name")
                    continue
                alreadyAdded[actor['name']] = True
                if actor['name'] in self.actorMap:
                    update_actors.append(actor)
                else:
                    self.task_count += 1
                    self._spawn(actor=actor, callback=complete)
            pcount = 0
            for key, value in self.actorMap.items():
                if key not in alreadyAdded:
                    alreadyAdded[key] = True
                    update_actors.append({
                        "name": key,
                        "module": "pedestrian.pool",
                        "params": {
                            "init_x": float(pcount),
                            "init_y": 10.0,
                            "init_a": -90.0,
                        },
                    })
                    pcount += 1
        if self.task_count == 0:
            def complete1(future):
                logging.debug(f"done complete1 {future.result()}")
                if callback:
                    callback(future)
            self._update(actors=update_actors, callback=complete1)

    def _spawn(self, actor=None, callback=None):
        name = actor['name'] if 'name' in actor else uuid
        module = actor['module'] if 'module' in actor else "pedestrian.pool"
        params = actor['params'] if 'params' in actor else {}
        animation = actor['animation'] if 'animation' in actor else "walk.dae"
        params_xml = ""
        for k, v in params.items():
            t = identify_variable_type(v)
            params_xml += f"<{k} type=\"{t}\">{v}</{k}>"
        self.actorMap[name] = actor
        xx = len(self.actorMap)
        yy = 10
        actor_xml =         f"""
<?xml version="1.0" ?>
<sdf version="1.6">
    <actor name="{name}">
        <pose>{xx} {yy} 0 0 0 0</pose>
        <skin>
            <filename>{animation}</filename>
            <scale>1.0</scale>
        </skin>
        <animation name="walking">
            <filename>{animation}</filename>
            <scale>1.0</scale>
            <interpolate_x>true</interpolate_x>
        </animation>
        <plugin name="pedestrian_plugin_{name}" filename="libpedestrian_plugin.so">
          <module>{module}</module>
          <robot>mobile_base</robot>
          {params_xml}
        </plugin>
    </actor>
</sdf>
"""
#         f"""
# <?xml version="1.0" ?>
# <sdf version="1.6">
#     <actor name="{name}">
#         <pose>{xx} {yy} 0 0 0 0</pose>
#         <skin>
#             <filename>walk.dae</filename>
#             <scale>1.0</scale>
#         </skin>
#         <animation name="walking">
#             <filename>walk.dae</filename>
#             <scale>1.0</scale>
#             <interpolate_x>true</interpolate_x>
#         </animation>
#         <plugin name="pedestrian_plugin_{name}" filename="libpedestrian_plugin.so">
#           <module>{module}</module>
#           <robot>mobile_base</robot>
#           {params_xml}
#         </plugin>
#     </actor>
# </sdf>
# """
        # logging.info(actor_xml)
        request = SpawnEntity.Request()
        request.name = name
        request.xml = actor_xml
        request.reference_frame = "world"
        future = self.spawn_entity_client.call_async(request)
        self.futures[uuid.uuid4()] = future

        def complete(future):
            if callback:
                callback(future)
            self.actorMap[name] = actor
        future.add_done_callback(complete)

    def _update(self, actors=None, callback=None):
        request = PluginUpdate.Request()
        for actor in actors:
            msg = Plugin()
            if 'name' not in actor or 'module' not in actor:
                logging.error("'name' and 'module' keys should be specified")
                continue
            msg.name = actor['name']
            msg.module = actor['module']
            if 'params' in actor:
                for key, value in actor['params'].items():
                    pMsg = PluginParam()
                    pMsg.name = key
                    pMsg.type = identify_variable_type(value)
                    pMsg.value = str(value)
                    msg.params.append(pMsg)
            request.plugins.append(msg)

        logging.debug(f"calling pedestrian_plugin_update {request}")
        future = self.pedestrian_plugin_update_client.call_async(request)
        self.futures[uuid.uuid4()] = future

        def done_callback(future):
            result = future.result()
            logging.debug(f"pedestrian_plugin_update service done: {result}")
            for name in result.plugin_names:
                if name not in self.actorMap:
                    self.actorMap[name] = {"name": name}
            if callback:
                callback(future)
        future.add_done_callback(done_callback)
