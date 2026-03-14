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

import logging
import uuid
import glob
import os
import random
import math
import re

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from pedestrian_plugin_msgs.msg import Agents
from pedestrian_plugin_msgs.msg import Plugin
from pedestrian_plugin_msgs.msg import PluginParam
from pedestrian_plugin_msgs.srv import PluginUpdate


def get_hips_rotation(dae_path):
    try:
        with open(dae_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Locate Hips node start
        hips_match = re.search(r'<node [^>]*id="Hips"[^>]*>', content)
        if not hips_match:
            return None, None
            
        start_idx = hips_match.end()
        # Find the next matrix element
        matrix_match = re.search(r'<matrix[^>]*>(.*?)</matrix>', content[start_idx:], re.DOTALL)
        if not matrix_match:
            return None, None
            
        matrix_str = matrix_match.group(1).strip()
        values = [float(x) for x in matrix_str.split()]
        
        if len(values) != 16:
            return None, None
            
        # m5 = cos(theta), m9 = sin(theta) for X-rotation
        # row-major: 0, 1, 2, 3 / 4, 5, 6, 7 / 8, 9, 10, 11
        m5 = values[5]
        m9 = values[9]
        
        theta = math.atan2(m9, m5)
        
        # Translation components (tx, ty, tz)
        translation = [values[3], values[7], values[11]]
        
        return theta, translation
    except Exception as e:
        logging.error(f"Error parsing DAE file {dae_path}: {e}")
        return None, None


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
        self.actorMap = {}
        self.futures = {}
        self.spawn_index = 0
        self.spawn_service_checked = False
        self._pending_actor_names = None   # set of actor names awaited via /human_states
        self._pending_actor_callback = None  # called when all pending actors appear
        
        self.models = []
        self.child_models = []
        self.adult_models = []
        
        # Try to find models from environment or default paths
        model_paths = [
            "/home/developer/models/LIRS-HMLG",
            os.path.expanduser("~/models/LIRS-HMLG"),
            os.path.join(os.getcwd(), "LIRS-HMLG"),
            "/home/ai-suitcase-1/nitta_workspace/cabot/cabot-navigation/LIRS-HMLG" # Fallback for host execution
        ]
        
        selected_path = None
        for path in model_paths:
            if os.path.exists(path):
                selected_path = path
                break
        
        if selected_path:
            self.node.get_logger().info(f"Loading models from {selected_path}")
            # Load adult models. Use glob.glob recursively if needed but here structure is known
            # Check directory structure matching: <root>/Male/<ModelName>/walk.dae
            self.adult_models.extend(glob.glob(os.path.join(selected_path, "Male", "*", "walk.dae")))
            self.adult_models.extend(glob.glob(os.path.join(selected_path, "Female", "*", "walk.dae")))
            
            # Load child models: <root>/Children/self_made/<ModelName>/*_walk.dae
            # Note: The pattern might be deeper or slightly different.
            self.child_models.extend(glob.glob(os.path.join(selected_path, "Children", "self_made", "*", "*_walk.dae")))
            
            self.models = self.adult_models + self.child_models
            self.node.get_logger().info(f"Found {len(self.adult_models)} adult models and {len(self.child_models)} child models")
        else:
            self.node.get_logger().error(f"Could not find LIRS-HMLG models in any of {model_paths}")

        if not self.models:
            self.node.get_logger().warn("No custom models found, will use default walk.dae")

    def human_states_callback(self, msg):
        if len(self.actorMap) < len(msg.agents):
            for agent in msg.agents:
                if agent.name not in self.actorMap:
                    self.actorMap[agent.name] = {}

        # When spawning new actors, detect readiness via /human_states instead of
        # relying on spawn_entity service responses (which can be dropped or delayed
        # when many concurrent requests are outstanding).
        if self._pending_actor_names is not None:
            current_names = {agent.name for agent in msg.agents}
            if self._pending_actor_names.issubset(current_names):
                actor_callback = self._pending_actor_callback
                self._pending_actor_names = None
                self._pending_actor_callback = None
                logging.debug(f"All pending actors appeared in /human_states, firing callback")
                if actor_callback:
                    actor_callback()

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

        # Reset any pending state from a previous (possibly timed-out) call.
        self._pending_actor_names = None
        self._pending_actor_callback = None

        update_actors = []
        new_actors = []

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
                    new_actors.append(actor)
                    # Fire-and-forget: spawn the entity but do not wait for the
                    # service response (responses can be dropped by DDS when many
                    # concurrent requests are outstanding).  Completion is detected
                    # via /human_states instead (see human_states_callback).
                    self._spawn(actor=actor, callback=None)
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

        if new_actors:
            # Spawn new actors and wait for them to appear in /human_states.
            # Simultaneously send pool-update for existing actors IMMEDIATELY so
            # that old actors are removed from the simulation early, reducing the
            # total number of concurrently active actors and preventing Gazebo
            # from being overloaded by two full sets of actors running at once.
            new_actor_names = {actor['name'] for actor in new_actors}
            logging.debug(f"Waiting for {len(new_actor_names)} new actors in /human_states: {new_actor_names}")

            # Two conditions must BOTH be satisfied before calling the completion callback:
            #   [0] all new actors have appeared in /human_states
            #   [1] pool-update service response received for existing actors
            #        (skipped immediately when update_actors is empty)
            completion_flags = [False, False]
            saved_future = [None]

            def try_complete():
                if completion_flags[0] and completion_flags[1]:
                    if callback:
                        callback(saved_future[0])

            def on_all_spawned():
                completion_flags[0] = True
                try_complete()

            self._pending_actor_names = new_actor_names
            self._pending_actor_callback = on_all_spawned

            if update_actors:
                # Send pool-update immediately to get a proper ROS future and to
                # move old actors to pool as soon as possible.
                def pool_done(future):
                    saved_future[0] = future
                    completion_flags[1] = True
                    try_complete()

                self._update(actors=update_actors, callback=pool_done)
            else:
                # No existing actors to pool-update; skip the service call and
                # mark that condition as already satisfied so that the completion
                # callback fires as soon as all new actors appear in /human_states.
                completion_flags[1] = True
        else:
            # All actors already existed; just send parameter updates.
            def complete1(future):
                logging.debug(f"done complete1 {future.result()}")
                if callback:
                    callback(future)
            self._update(actors=update_actors, callback=complete1)

    def _spawn(self, actor=None, callback=None):
        name = actor['name'] if 'name' in actor else uuid.uuid4()
        module = actor['module'] if 'module' in actor else "pedestrian.pool"
        params = actor['params'] if 'params' in actor else {}
        params_xml = ""
        for k, v in params.items():
            t = identify_variable_type(v)
            if t == "bool":
                v = "true" if v else "false"
            elif t == "list":
                v = " ".join([str(x) for x in v])
            params_xml += f"<{k} type='{t}'>{v}</{k}>\n"
        self.actorMap[name] = actor

        self.spawn_index += 1
        xx = self.spawn_index
        yy = 10
        
        skin_file = "walk.dae"
        animation_file = "walk.dae"
        
        is_child = "child" in name.lower() or "child" in module.lower()
        
        selected_models = []
        if is_child and self.child_models:
             selected_models = self.child_models
        elif not is_child and self.adult_models:
             selected_models = self.adult_models
             
        if selected_models:
            skin_file = random.choice(selected_models)
            # Use the same file for animation to avoid skeleton mismatch
            # If the model doesn't support animation, it will slide, which is better than distortion
            animation_file = skin_file

        # corrections for LIRS models (visual offset in skin, not actor)
        # Actor pose (main movement axis) should be flat
        pose_z = 0.0
        pose_r = 0.0
        pose_p = 0.0
        pose_y = 0.0
        # Skin pose (visual correction relative to actor)
        skin_z = 0.0
        skin_r = 0.0
        skin_p = 0.0
        skin_y = 0.0
        
        if skin_file != "walk.dae":
            # Determine appropriate rotation based on DAE file analysis
            theta, translation = get_hips_rotation(skin_file)
            
            # Apply correction to ACTOR pose (pose_r), as skin pose is ignored
            if theta is not None:
                pose_r = theta - 1.57
                
                # Dynamic Z correction
                if translation:
                    tx, ty, tz = translation
                    pose_z = tz - 1.05
            else:
                 # Fallback if parsing fails
                if "Children" in skin_file:
                    pose_r = -0.96
                    pose_z = -0.15
                elif "Female" in skin_file:
                    pose_r = -1.35
                    pose_z = -0.05
                else: 
                    pose_r = -1.57
                    pose_z = -0.05

        actor_xml = f"""
<?xml version="1.0" ?>
<sdf version="1.6">
    <actor name="{name}">
        <pose>{xx} {yy} {pose_z} {pose_r} {pose_p} {pose_y}</pose>
        <skin>
            <filename>{skin_file}</filename>
            <scale>1.0</scale>
            <pose>0 0 {skin_z} {skin_r} {skin_p} {skin_y}</pose>
        </skin>
        <animation name="walking">
            <filename>{animation_file}</filename>
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
        logging.debug(actor_xml)
        
        if not self.spawn_service_checked:
            if not self.spawn_entity_client.wait_for_service(timeout_sec=10.0):
                logging.error(f"spawn_entity service not available")
                # Ensure callback is called to prevent hanging
                if callback:
                    # Create a dummy future or just call with None
                    from rclpy.task import Future
                    dummy_future = Future()
                    dummy_future.set_result(None)
                    callback(dummy_future)
                return
            self.spawn_service_checked = True

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