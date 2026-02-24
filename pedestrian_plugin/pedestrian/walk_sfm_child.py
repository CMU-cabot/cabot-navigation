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

import ros
import random
import json
from pedestrian import walk_sfm

child_states = {} # map from actor name to child specific state

def onUpdate(**args):
    name = args['name']
    dt = args['dt']

    # Child specific parameters
    goal_candidates = args.get('goals', []) # List of [x, y]
    if isinstance(goal_candidates, str):
        try:
            goal_candidates = json.loads(goal_candidates)
        except Exception as e:
            ros.info(f"Error parsing goals: {e}")
            goal_candidates = []

    velocity_range = args.get('velocity_range', [0.5, 1.5])
    if isinstance(velocity_range, str):
        try:
            velocity_range = json.loads(velocity_range)
        except Exception as e:
            ros.info(f"Error parsing velocity_range: {e}")
            velocity_range = [0.5, 1.5]

    change_prob = args.get('change_probability', 1.0)
    change_interval_min = args.get('change_interval_min', 3.0)
    change_interval_max = args.get('change_interval_max', 5.0)

    # Detect test-case transition: if stored goals differ from current goals,
    # clear state so the actor re-initializes for the new scenario.
    if name in child_states:
        # Check if goals changed (simple identity check list vs list content?)
        # Since goal_candidates is freshly parsed from args, specific object changes but value should be same.
        if child_states[name].get('goals') != goal_candidates:
            ros.info(f"Child {name}: goals changed, resetting state")
            del child_states[name]

    # Initialize or update child state
    if name not in child_states:
        child_states[name] = {
            'elapsed_time': 0.0,
            'current_goal_idx': -1,
            'current_velocity': args.get('velocity', 1.0),
            'current_interval': random.uniform(change_interval_min, change_interval_max),
            'goals': goal_candidates,
        }

        # Initialize goal if available
        if goal_candidates:
            child_states[name]['current_goal_idx'] = 0
            if len(goal_candidates) > 1 and random.random() < change_prob:
                 child_states[name]['current_goal_idx'] = random.randint(0, len(goal_candidates)-1)

    c_state = child_states[name]
    c_state['elapsed_time'] += dt
    
    # Logic to change goal and velocity
    if c_state['elapsed_time'] > c_state['current_interval']:
        c_state['elapsed_time'] = 0.0
        
        # Reset interval for next cycle
        c_state['current_interval'] = random.uniform(change_interval_min, change_interval_max)
        
        # Determine if we should change behavior
        if random.random() < change_prob:
            # Change Goal
            if len(goal_candidates) > 0:
                # Pick a random goal index different from current if possible
                if len(goal_candidates) > 1:
                    candidates = [i for i in range(len(goal_candidates)) if i != c_state['current_goal_idx']]
                    new_idx = random.choice(candidates)
                else:
                    new_idx = 0
                c_state['current_goal_idx'] = new_idx
                #ros.info(f"Child {name} changed goal to index {new_idx}")

            # Change Velocity
            c_state['current_velocity'] = random.uniform(velocity_range[0], velocity_range[1])
            ros.info(f"Child {name} changed velocity to {c_state['current_velocity']:.2f}")

    # Set parameters for SFM
    if c_state['current_goal_idx'] >= 0 and c_state['current_goal_idx'] < len(goal_candidates):
        target = goal_candidates[c_state['current_goal_idx']]
        args['goal_x'] = target[0]
        args['goal_y'] = target[1]

    # Override velocity with the child's current dynamic velocity
    args['velocity'] = c_state['current_velocity']

    # Delegate to shared walk_sfm
    return walk_sfm.onUpdate(**args)
