#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2024  IBM Corporation
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
###############################################################################

# metrics implementation is based on Hunav Evaluator (MIT license)
# https://github.com/robotics-upo/hunav_sim/tree/humble/hunav_evaluator

import numpy as np
import math
import rclpy
from pedestrian_plugin_msgs.msg import Agent
from geometry_msgs.msg import Pose

# Teaching Robot Navigation Behaviors to Optimal RRT Planners
# Noé Pérez-Higueras, Fernando Caballero & Luis Merino


def euclidean_distance(pose, pose1):
    return math.sqrt((pose.position.x - pose1.position.x)**2 + (pose.position.y - pose1.position.y)**2)


def get_group_center(agents_i, group_id, distance):

    group = []
    for agent in agents_i:
        if agent.group_id == group_id:
            pose = Pose()
            pose.position.x = agent.position.position.x + (distance * math.cos(agent.yaw))
            pose.position.y = agent.position.position.y + (distance * math.sin(agent.yaw))
            group.append(pose)

    interaction_center = Pose()
    for p in group:
        interaction_center.position.x += p.position.x
        interaction_center.position.y += p.position.y

    interaction_center.position.x = float(interaction_center.position.x/len(group))
    interaction_center.position.y = float(interaction_center.position.y/len(group))
    return interaction_center


def indicator_function(norm, k):
    if k == 'intimate':
        if norm < 0.45:
            return 1
        else:
            return 0
    elif k == 'personal':
        if norm >= 0.45 and norm < 1.2:
            return 1
        else:
            return 0
    elif k == 'social':
        if norm >= 1.2 and norm < 3.6:
            return 1
        else:
            return 0
    elif k == 'public':
        if norm >= 3.6:
            return 1
        else:
            return 0
    else:
        return 0


def get_time_stamps(agents, robot):
    time_list = []
    t0 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    for a in agents:
        t = rclpy.time.Time.from_msg(a.header.stamp)
        dur = (t - t0).to_msg()
        s = float(dur.sec + dur.nanosec/1e9)
        time_list.append(s)
    return time_list


def total_time(agents, robot):
    t2 = rclpy.time.Time.from_msg(agents[len(agents)-1].header.stamp)
    t1 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    dur = (t2 - t1).to_msg()
    secs = float(dur.sec + dur.nanosec/1e9)
    return [secs]


def robot_path_length(agents, robot, accumulation_threshold=0.01):
    path_length = 0.0
    prev_position = robot[0].position
    for i in range(len(robot)-1):
        distance = euclidean_distance(robot[i+1].position, prev_position)
        # ignore small changes
        if accumulation_threshold <= distance:
            path_length += distance
            prev_position = robot[i+1].position
    return [path_length]


def cumulative_heading_changes(agents, robot, accumulation_threshold=math.radians(1)):
    chc_list = [0.0]
    chc = 0
    prev_yaw = robot[0].yaw
    for i in range(len(robot) - 1):
        norm = normalize_angle(prev_yaw - robot[i+1].yaw)
        if norm < 0.0:
            norm *= -1
        # ignore small changes
        if accumulation_threshold <= norm:
            prev_yaw = robot[i+1].yaw
        else:
            norm = 0.0
        chc += norm
        chc_list.append(norm)

    return [chc, chc_list]


def normalize_angle(ang):
    while (ang <= -math.pi):
        ang += 2 * math.pi
    while (ang > math.pi):
        ang -= 2 * math.pi
    return ang


def avg_closest_person(agents, robot):
    min_dist_list = []
    avg_dist = 0
    for i in range(len(robot)):
        min_dist = 10000
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius
            if (d < min_dist):
                min_dist = d
                if min_dist < 0.0:
                    min_dist = 0.0
        if (len(agents[i].agents) > 0):
            avg_dist += min_dist
            min_dist_list.append(min_dist)

    avg_dist = avg_dist/len(robot)
    return [avg_dist, min_dist_list]


def minimum_distance_to_people(agents, robot):
    min_distance = list()
    for i in range(len(robot)):
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius
            if d < 0.0:
                d = 0.0
            min_distance.append(d)
    min_dist = min(min_distance)
    return [min_dist]


def maximum_distance_to_people(agents, robot):
    max_distance = list()
    for i in range(len(robot)):
        for agent in agents[i].agents:
            max_distance.append(euclidean_distance(robot[i].position, agent.position) - robot[i].radius)
    max_dist = max(max_distance)
    return [max_dist]


def space_intrusions(agents, robot, k):
    space_intrusions = 0
    space_intrusions_list = [0] * len(robot)

    for i in range(len(robot)):
        min_dist = 10000
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius
            if d < min_dist:
                min_dist = d
                if min_dist < 0.0:
                    min_dist = 0.0
        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1
            space_intrusions_list[i] = 1

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0

    return percentage, space_intrusions_list


def intimate_space_intrusions(agents, robot):
    percentage, slist = space_intrusions(agents, robot, 'intimate')
    return [percentage, slist]


def personal_space_intrusions(agents, robot):
    percentage, slist = space_intrusions(agents, robot, 'personal')
    return [percentage, slist]


def social_space_intrusions(agents, robot):
    percentage, slist = space_intrusions(agents, robot, 'social')
    return [percentage, slist]


def detect_groups(agents):
    group_ids = []
    for a in agents[0].agents:
        if (a.group_id != -1 and ((a.group_id in group_ids) is False)):
            group_ids.append(a.group_id)

    return group_ids


def group_space_intrusions(agents, robot, k):
    group_ids = detect_groups(agents)
    if (len(group_ids) == 0):
        return [0.0]

    d = 1.5
    space_intrusions = 0
    group_list = [0] * len(robot)
    for i in range(len(robot)):
        min_dist = 10000
        for id in group_ids:
            group_center = get_group_center(agents[i].agents, id, d)
            dist = euclidean_distance(robot[i].position, group_center.position) - robot[i].radius
            if dist < min_dist:
                min_dist = dist
        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1
            group_list[i] = 1

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0

    return [percentage, group_list]


def group_intimate_space_intrusions(agents, robot):
    r = group_space_intrusions(agents, robot, 'intimate')
    return r


def group_personal_space_intrusions(agents, robot):
    r = group_space_intrusions(agents, robot, 'personal')
    return r


def group_social_space_intrusions(agents, robot):
    r = group_space_intrusions(agents, robot, 'social')
    return r


# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

# The metrics Robot on Person Personal Distance Violation, Person on Robot Personal Distance Violation, Intimate Distance Violation and
# Person on Robot Intimate Distance Violation have already been implemented in the Personal_space_intrusions function.
# Instead of returning the number of times, it returns a percentage of distance violation.

def collisions(agents, robot):
    robot_coll_list = [0] * len(robot)
    person_coll_list = [0] * len(robot)
    robot_collisions = 0
    person_collisions = 0

    for i in range(len(robot)):
        for agent in agents[i].agents:

            if euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius < 0.02:

                # Robot's angle
                nrx = (robot[i].position.position.x - agent.position.position.x) * math.cos(agent.yaw) + (robot[i].position.position.y - agent.position.position.y) * math.sin(agent.yaw)
                nry = -(robot[i].position.position.x - agent.position.position.x) * math.sin(agent.yaw) + (robot[i].position.position.y - agent.position.position.y) * math.cos(agent.yaw)
                alpha = math.atan2(nry, nrx)

                # Agent's angle
                nrx = (agent.position.position.x - robot[i].position.position.x) * math.cos(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.sin(robot[i].yaw)
                nry = -(agent.position.position.x - robot[i].position.position.x) * math.sin(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.cos(robot[i].yaw)
                alpha2 = math.atan2(nrx, nry)

                if abs(alpha) < abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
                    robot_collisions += 1
                    robot_coll_list[i] = 1
                elif abs(alpha) > abs(alpha2) and robot[i].linear_vel < agent.linear_vel:
                    person_collisions += 1
                    person_coll_list[i] = 1
                elif abs(alpha) < abs(alpha2) and robot[i].linear_vel < agent.linear_vel:
                    # person_collision += 1
                    robot_collisions += 1
                    robot_coll_list[i] = 1
                elif abs(alpha) > abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
                    # robot_collision += 1
                    person_collisions += 1
                    person_coll_list[i] = 1
                elif abs(alpha) == abs(alpha2) and robot[i].linear_vel == agent.linear_vel:
                    robot_collisions += 1
                    person_collisions += 1
                    robot_coll_list[i] = 1
                    person_coll_list[i] = 1

    return robot_collisions, person_collisions, robot_coll_list, person_coll_list


def robot_on_person_collision(agents, robot):
    collision = collisions(agents, robot)
    return [collision[0], collision[2]]


def person_on_robot_collision(agents, robot):
    collision = collisions(agents, robot)
    return [collision[1], collision[3]]


def time_not_moving(agents, robot, linear_vel_threshold=0.01, angular_vel_threshold=0.02):
    time_stopped = 0
    not_moving = [0]*len(robot)

    stamps = get_time_stamps(agents, robot)

    for i in range(len(robot)):
        r = robot[i]

        if i == 0:
            dt = (stamps[i+1] - stamps[i])/2.0
        elif i == len(robot)-1:
            dt = (stamps[i] - stamps[i-1])/2.0
        else:
            dt = (stamps[i+1] - stamps[i])/2.0 + (stamps[i] - stamps[i-1])/2.0

        if (r.linear_vel < linear_vel_threshold and abs(r.angular_vel < angular_vel_threshold)):
            time_stopped += dt
            not_moving[i] = 1

    return [time_stopped, not_moving]


def goal_reached(agents, robot):
    mind = 0.0
    if (len(robot[-1].goals)):
        for g in robot[-1].goals:
            d = euclidean_distance(robot[-1].position, g) - robot[-1].goal_radius
            if d < mind:
                return [True]
    return [False]


def final_goal_distance(agents, robot):
    min_dist = 10000
    if (len(robot[-1].goals)):
        for g in robot[-1].goals:
            d = euclidean_distance(robot[-1].position, g)
            if d < min_dist:
                min_dist = d
    return [min_dist]


def minimum_goal_distance(agents, robot):
    min_dist = 10000
    for r in robot:
        if (len(r.goals)):
            for g in r.goals:
                d = euclidean_distance(r.position, g)
                if d < min_dist:
                    min_dist = d
    return [min_dist]


# collision count based on the collision criteria implemented in SEAN 2.0
# https://github.com/yale-sean/social_sim_unity/blob/main/Assets/Scripts/SEAN/Metrics/CountCollisions.cs

def collision_count(agents, robot):
    return count_collisions_sean2(agents, robot)


def robot_on_person_collision_count(agents, robot):
    return count_robot_on_person_collisions_sean2(agents, robot)


def person_on_robot_collision_count(agents, robot):
    return count_person_on_robot_collisions_sean2(agents, robot)


def count_collisions_sean2(agents, robot):
    robot_coll_list = [0] * len(robot)
    person_coll_list = [0] * len(robot)
    robot_collisions = 0
    person_collisions = 0

    # variable to count independent collisions
    agents_in_collision = {}

    for i in range(len(robot)):
        for agent in agents[i].agents:
            agent_name = agent.name

            if euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius <= 0.0:  # exact collision couting without margin
                in_collision = agents_in_collision.get(agent_name, False)

                # agent angle with respect to the robot orientation (yaw)
                nrx = (agent.position.position.x - robot[i].position.position.x) * math.cos(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.sin(robot[i].yaw)
                nry = -(agent.position.position.x - robot[i].position.position.x) * math.sin(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.cos(robot[i].yaw)
                alpha2 = math.atan2(nry, nrx)  # math.atan2(y, x)

                # collision criteria based on SEAN 2.0
                # If the robot isn't moving, it can't be at fault
                vel = round(robot[i].linear_vel, 3)
                # If the robot is oriented towards the other agent
                oriented_towards_other = np.abs(alpha2) < np.radians(90.0)
                is_robot_at_fault = (vel != 0.0) and oriented_towards_other

                if not in_collision:
                    if is_robot_at_fault:
                        robot_collisions += 1
                        robot_coll_list[i] += 1
                    else:
                        person_collisions += 1
                        person_coll_list[i] += 1

                agents_in_collision[agent_name] = True
            else:
                agents_in_collision[agent_name] = False

    return robot_collisions, person_collisions, robot_coll_list, person_coll_list


def count_robot_on_person_collisions_sean2(agents, robot):
    collision = count_collisions_sean2(agents, robot)
    return [collision[0], collision[2]]


def count_person_on_robot_collisions_sean2(agents, robot):
    collision = count_collisions_sean2(agents, robot)
    return [collision[1], collision[3]]


# SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
# ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University

def avg_robot_linear_speed(agents, robot):
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.linear_vel)
        speed += r.linear_vel

    speed = speed / len(robot)

    return [speed, speed_list]


def avg_robot_angular_speed(agents, robot):
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.angular_vel)
        speed += np.abs(r.angular_vel)

    speed = speed / len(robot)

    return [speed, speed_list]


def avg_acceleration(agents, robot):
    acceleration = 0
    acceleration_list = [0.0]
    for i in range(len(robot) - 1):
        dv = robot[i+1].linear_vel - robot[i].linear_vel
        tf = rclpy.time.Time.from_msg(agents[i+1].header.stamp)
        ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
        dur = (tf - ti).to_msg()
        dt = float(dur.sec + dur.nanosec/1e9)
        if dt != 0.0:
            accel = dv/dt
            acceleration += np.abs(accel)
            acceleration_list.append(accel)
        else:
            acceleration_list.append(0.0)

    acceleration = acceleration / len(robot)

    return [acceleration, acceleration_list]


def avg_overacceleration(agents, robot):
    jerk = 0
    jerk_list = [0.0]
    for i in range(len(robot) - 1):
        dv = robot[i+1].linear_vel - robot[i].linear_vel
        tf = rclpy.time.Time.from_msg(agents[i+1].header.stamp)
        ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
        dur = (tf - ti).to_msg()
        dt = float(dur.sec + dur.nanosec/1e9)
        if dt != 0.0:
            acceleration = dv/dt
            jerk += np.abs(acceleration/dt)
            jerk_list.append(acceleration/dt)
        else:
            jerk_list.append(0.0)

    jerk = jerk / len(robot)

    return [jerk, jerk_list]


# Learning a Group-Aware Policy for Robot Navigation
# Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 , Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2

def avg_pedestrian_velocity(agents, robot):
    speed = 0
    speed_list = []
    for i in range(len(agents)):
        speed2 = 0.0
        for agent in agents[i].agents:
            speed += agent.linear_vel
            speed2 += agent.linear_vel
        speed_list.append(speed2/len(agents[i].agents))

    speed = speed / (len(agents) * len(agents[0].agents))

    return [speed, speed_list]


def avg_closest_pedestrian_velocity(agents, robot):
    speed = 0
    speed_list = []
    for i in range(len(robot)):
        min_dist = 10000
        closest = Agent()
        for agent in agents[i].agents:
            d = euclidean_distance(robot[i].position, agent.position)
            if (d < min_dist):
                min_dist = d
                closest = agent
                if min_dist < 0.0:
                    min_dist = 0.0

        speed += closest.linear_vel
        speed_list.append(closest.linear_vel)

    speed = speed/len(robot)
    return [speed, speed_list]


def avg_pedestrian_angle(agents, robot):
    pass


# metrics based on social force model

# # cumulative modulus of the social force provoked
# # by the robot in the agents
# def social_force_on_agents(agents, robot):
#     sfm = SFM()
#     sf = 0.0
#     sf_list = []
#     for agts, rb in zip(agents, robot):
#         f = sfm.modulusSocialForce2(rb, agts)
#         sf += f
#         sf_list.append(f)
#     return [sf, sf_list]

# # cumulative modulus of the social force provoked
# # by the agents in the robot
# def social_force_on_robot(agents, robot):
#     sfm = SFM()
#     sf = 0.0
#     sf_list = []
#     for agts, rb in zip(agents, robot):
#         f = sfm.modulusSocialForce(rb, agts)
#         sf += f
#         sf_list.append(f)
#     return [sf, sf_list]

# # cumulative social work employed in this planner:
# # https://github.com/robotics-upo/social_force_window_planner
# def social_work(agents, robot):
#     sfm = SFM()
#     sw = 0.0
#     sw_list = []
#     for agts, rb in zip(agents, robot):
#         f = sfm.computeSocialWork(rb, agts)
#         sw += f
#         sw_list.append(f)
#     return [sw, sw_list]


# # cumulative obstacle force on the agents
# def obstacle_force_on_agents(agents, robot):
#     sfm = SFM()
#     of = 0.0
#     of_list = []
#     for agts in agents:
#         avg = 0.0
#         for a in agts.agents:
#             avg += np.linalg.norm(sfm.computeObstacleForce(a))
#         avg =avg/len(agts.agents)
#         of += avg
#         of_list.append(avg)
#     return [of, of_list]


# # cumulative obstacle force on the robot
# def obstacle_force_on_robot(agents, robot):
#     sfm = SFM()
#     of = 0.0
#     of_list = []
#     for r in robot:
#         f = np.linalg.norm(sfm.computeObstacleForce(r))
#         of += f
#         of_list.append(f)
#     return [of, of_list]


# TODO
def path_irregularity(agents, robot):
    pass


# TODO
def path_efficiency(agents, robot):
    pass


# TODO
def static_obs_collision(agents, robot):
    pass


# Evaluation of Socially-Aware Robot Navigation
# Yuxiang Gao * and Chien-Ming Huang
# Department of Computer Science, The Johns Hopkins University, Baltimore, MD, United States


# TODO:
# Average Displacement Error --> Trajectory needed
# Final Displacement Error --> Trajectory needed
# Asymmetric Dynamic Time Warping --> Trajectory needed
# Topological Complexity --> Path needed
# Path irregularity and Path efficiency are similar to the ones in SEAN paper .
# Personal space and o/p/r-space metrics are similar to the ones in Teaching Robot Navigation Behaviors to Optimal RRT Planners paper.


metrics = {
    # N. Perez-Higueras, F. Caballero, and L. Merino, “Teaching Robot Nav-
    # igation Behaviors to Optimal RRT Planners,” International Journal of
    # Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.
    'time_to_reach_goal': total_time,
    'path_length': robot_path_length,
    'cumulative_heading_changes': cumulative_heading_changes,
    'avg_distance_to_closest_person': avg_closest_person,
    'minimum_distance_to_people': minimum_distance_to_people,
    'maximum_distance_to_people': maximum_distance_to_people,
    'intimate_space_intrusions': intimate_space_intrusions,
    'personal_space_intrusions': personal_space_intrusions,
    'social_space_intrusions': social_space_intrusions,
    'group_intimate_space_intrusions': group_intimate_space_intrusions,
    'group_personal_space_intrusions': group_personal_space_intrusions,
    'group_social_space_intrusions': group_social_space_intrusions,
    # N. Tsoi, A. Xiang, P. Yu, S. S. Sohn, G. Schwartz, S. Ramesh,
    # M. Hussein, A. W. Gupta, M. Kapadia, and M. V ́azquez, “Sean 2.0:
    # Formalizing and generating social situations for robot navigation,”
    # IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11 047–
    # 11 054, 2022
    #   - 'Total Path Length' (meters): similar to 'path_length'
    #   - 'Path Irregularity': (radians): total rotations in the robot's
    #       traveled path greater than the total rotations in the search-based
    #       path from the starting pose.
    #   - 'Path Efficiency': (meters): ratio between robot's traveled path and
    #       geodesic distance of the search-based path from the starting pose.

    # true when the robot's final pose is within a specified distance of the goal.
    # The final distance threshold is easily adjustable by the user, but defaults
    # to 1.2m.
    'completed': goal_reached,
    # (meters): the closest the robot passes to the target position.
    'minimum_distance_to_target': minimum_goal_distance,
    # (meters): distance between the last robot position and the target position.
    'final_distance_to_target': final_goal_distance,
    #   - 'Robot on Person Personal Distance Violation': number of times a robot
    # approaches a person within the personal distance of the robot.
    # Similar to 'personal_space_intrusions'
    #   - 'Person on Robot Personal Distance Violation': number of times a person
    # approaches the robot within the personal distance of the robot.
    #   - 'Intimate Distance Violation': number of times the robot approached within
    # the intimate distance of a person.
    #   - 'Person on Robot Intimate Distance Violation': number of times a person
    # approaches the robot within the intimate distance of the robot.
    'robot_on_person_collision': robot_on_person_collision,
    'person_on_robot_collision': person_on_robot_collision,
    'time_not_moving': time_not_moving,
    # TODO: 'static_obstacle_collision': static_obs_collision,
    # number of times the robot collides with a static obstacle.

    # SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
    # ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University
    'avg_robot_linear_speed': avg_robot_linear_speed,
    'avg_robot_angular_speed': avg_robot_angular_speed,
    'avg_acceleration': avg_acceleration,
    'avg_overacceleration': avg_overacceleration,

    # Learning a Group-Aware Policy for Robot Navigation
    # Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 , Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2
    'avg_pedestrian_velocity': avg_pedestrian_velocity,
    'avg_closest_pedestrian_velocity': avg_closest_pedestrian_velocity,

    # metrics based on Social Force Model employed in different papers
    # 'social_force_on_agents': social_force_on_agents,
    # 'social_force_on_robot': social_force_on_robot,
    # 'social_work': social_work,
    # 'obstacle_force_on_robot': obstacle_force_on_robot,
    # 'obstacle_force_on_agents': obstacle_force_on_agents,
}
