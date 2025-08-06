#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2024  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import pytest
import numpy as np
import matplotlib.pyplot as plt
from mf_localization.altitude_manager import AltitudeFloorEstimator
from mf_localization.altitude_manager import BalancedSampler
from mf_localization.altitude_manager import FloorHeightMapper


class PressureSimulator:
    def __init__(self):
        self.p0: float = 101325.0  # [Pa]
        self.c0: float = 44300.0
        self.c1: float = 5.255

    def simulate_pressure(self, height):
        p = self.p0 * (1.0 - height/self.c0)**self.c1
        return p


def generate_samples(xmin, xmax, ymin, ymax, floor, area, height, effective_radius=np.nan, dx=1.0, dy=1.0):
    X = []
    for i in range(int((xmax-xmin)/dx)):
        for j in range(int((ymax-ymin)/dy)):
            x = xmin + i*dx
            y = ymin + j+dy
            X.append([x, y, floor, area, height, effective_radius])
    return X


def generate_all_samples():
    X = []  # [x, y, floor, area, height]

    # ground floor
    # floor 0, area 0, height = 0.0
    X_0 = generate_samples(xmin=0, xmax=40, ymin=0, ymax=10, floor=0, area=0, height=0)

    # building 1 (simple building)
    # floor -1, area 0, height = -4.0
    X_B1_0 = generate_samples(xmin=1, xmax=9, ymin=1, ymax=9, floor=-1, area=0, height=-4.5)
    # floor 1, area 0, height = 4.0
    X_1_0 = generate_samples(xmin=2, xmax=8, ymin=2, ymax=8, floor=1, area=0, height=4.0)
    # floor 2, area 0, height = 9.0
    X_2_0 = generate_samples(xmin=3, xmax=7, ymin=3, ymax=7, floor=2, area=0, height=9.0)

    # building 2 (high building)
    # floor 1, area 1, height = 12.0
    X_1_1 = generate_samples(xmin=15, xmax=20, ymin=2, ymax=8, floor=1, area=0, height=12.0)

    # building 3 (skip building)
    X_2_2 = generate_samples(xmin=25, xmax=35, ymin=1, ymax=9, floor=2, area=2, height=10.0)
    X_4_2 = generate_samples(xmin=28, xmax=32, ymin=3, ymax=7, floor=4, area=2, height=22.0)

    # building 4
    X_1_3 = generate_samples(xmin=2, xmax=8, ymin=15, ymax=25, floor=1, area=3, height=4.0)

    X = X_0 + X_B1_0 + X_1_0 + X_2_0 + X_1_1 + X_2_2 + X_4_2 + X_1_3
    return X


def plot_samples(X):
    plt.scatter(X[:, 0], X[:, 1], c=X[:, 4])
    plt.colorbar()
    plt.gca().set_aspect("equal")
    plt.show()


def test_floor_list():
    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)

    x = [5, 5]
    floor_list = floor_height_mapper.get_floor_list(x)
    assert floor_list == [-1, 0, 1, 2]

    x = [17, 5]
    floor_list = floor_height_mapper.get_floor_list(x)
    assert floor_list == [0, 1]

    x = [17.5, 5.5]
    floor_list = floor_height_mapper.get_floor_list(x, radius=0.01)
    assert floor_list == []


def test_height_list():
    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)

    x = [5, 5]
    height_list = floor_height_mapper.get_height_list(x)
    assert height_list == [-4.5, 0.0, 4.0, 9.0]

    x = [17.5, 5.5]
    height_list = floor_height_mapper.get_height_list(x)
    assert height_list == [0.0, 12.0]

    height_list = floor_height_mapper.get_height_list(x, radius=0.01)
    assert height_list == []


def test_altitude_floor_estimator_use_floor_height_list():
    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)
    altitude_floor_estimator = AltitudeFloorEstimator()

    # building 1
    x = [17, 5]
    floor_est = 0
    height_est = 0

    # default
    altitude_floor_estimator.reset(floor_est=floor_est, height_est=height_est)
    assert not altitude_floor_estimator.use_floor_height_list()

    # invalid list
    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    height_list[0] = np.nan
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)
    assert not altitude_floor_estimator.use_floor_height_list()

    # valid list
    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)
    assert altitude_floor_estimator.use_floor_height_list()

    # empty list
    floor_list, height_list = floor_height_mapper.get_floor_height_list(x=[1000, 1000])
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)
    assert not altitude_floor_estimator.use_floor_height_list()


def test_altitude_floor_estimator():
    # generic altitude floor estimator

    altitude_floor_estimator = AltitudeFloorEstimator()
    pressure_simulator = PressureSimulator()

    floor_est = 0
    height_est = 0

    altitude_floor_estimator.reset(floor_est=floor_est, height_est=height_est)

    timestamp = 0
    # floor = 0
    height = 0
    target_floor = 1
    target_height = 4.0
    dt = 0.5  # [s]
    vel_height = 0.5  # [m/s]

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor

    target_floor = 2
    target_height = 8.0
    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor

    target_floor = -1
    target_height = -4.0
    # down
    for i in range(int((target_height - height)/(-vel_height*dt))):
        timestamp += dt
        height += -vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor


def test_altitude_floor_estimator_building_1():
    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)
    altitude_floor_estimator = AltitudeFloorEstimator()
    pressure_simulator = PressureSimulator()

    # building 1
    x = [5, 5]
    floor_est = -1
    height_est = -4

    altitude_floor_estimator.reset(floor_est=floor_est, height_est=height_est)

    timestamp = 0
    # floor = -1
    height = -4
    target_floor = 1
    target_height = 4.0
    dt = 0.5  # [s]
    vel_height = 0.5  # [m/s]

    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor

    target_floor = 2
    target_height = 9.0
    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor


def test_altitude_floor_estimator_building_2_3():

    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)
    altitude_floor_estimator = AltitudeFloorEstimator()
    pressure_simulator = PressureSimulator()

    # building 2
    x = [17, 5]
    timestamp = 0
    # floor = 0
    height = 0
    floor_est = 0
    height_est = 0

    altitude_floor_estimator.reset(floor_est=floor_est, height_est=height_est)

    assert not altitude_floor_estimator.use_floor_height_list()

    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)

    assert altitude_floor_estimator.use_floor_height_list()

    target_floor = 1
    target_height = 12.0
    dt = 0.5  # [s]
    vel_height = 0.5  # [m/s]

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor

    # down
    target_floor = 0
    target_height = 0.0
    # down
    for i in range(int((target_height - height)/(-vel_height*dt))):
        timestamp += dt
        height += -vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor

    # building 3
    x = [30, 5]
    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)

    # up
    target_floor = 2
    target_height = 10.0
    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor


def test_altitude_floor_estimator_incomplete_floor_list():
    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)
    altitude_floor_estimator = AltitudeFloorEstimator()
    pressure_simulator = PressureSimulator()

    # outside of ground floor but in building 4
    x = [5, 20]
    floor_est = 0
    height_est = 0

    altitude_floor_estimator.reset(floor_est=floor_est, height_est=height_est)

    timestamp = 0
    # floor = 0
    height = 0
    target_floor = 1
    target_height = 4.0
    dt = 0.5  # [s]
    vel_height = 0.5  # [m/s]

    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    assert floor_list == [1.0]

    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)
    assert not altitude_floor_estimator.use_floor_height_list()

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    # up
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est = result.floor_est, result.height_est
        print(f"floor_est={floor_est}, height_est={height_est}")

    assert floor_est == target_floor


def test_altitude_floor_estimator_irregular_pressure_pattern():
    X = generate_all_samples()

    floor_height_mapper = FloorHeightMapper(X)
    altitude_floor_estimator = AltitudeFloorEstimator()
    pressure_simulator = PressureSimulator()

    x = [0, 0]
    floor_est = 0
    height_est = 0

    altitude_floor_estimator.reset(floor_est=floor_est, height_est=height_est)

    timestamp = 0
    floor = 0
    height = 0

    dt = 0.5  # [s]
    floor_list, height_list = floor_height_mapper.get_floor_height_list(x)
    altitude_floor_estimator.set_floor_height_list(floor_list, height_list)
    print(f"floor_list={floor_list}, height_list={height_list}")

    # stable
    for i in range(60):
        timestamp += dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est, current_state = result.floor_est, result.height_est, result.current_state
        print(f"height={height}, floor_est={floor_est}, height_est={height_est}, current_state={current_state}")

    target_height = -2.0
    vel_height = -1.0  # [m/s]

    # decrease height to enter DOWN state
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est, current_state = result.floor_est, result.height_est, result.current_state
        print(f"height={height}, floor_est={floor_est}, height_est={height_est}, current_state={current_state}")

    # rapidly increase height in DOWN state before floor change
    height = 0.0
    target_height = 1
    vel_height = 0.5
    for i in range(int((target_height - height)/(vel_height*dt))):
        timestamp += dt
        height += vel_height*dt
        pressure = pressure_simulator.simulate_pressure(height)
        result = altitude_floor_estimator._put_pressure(timestamp, pressure)
        floor_est, height_est, current_state = result.floor_est, result.height_est, result.current_state
        print(f"height={height}, floor_est={floor_est}, height_est={height_est}, current_state={current_state}")

    assert floor == floor_est


def test_balanced_sampler():
    sampler = BalancedSampler()

    # generate same values
    values = [1.0]
    probs = [1.0]
    sampler.update(x=values, p=probs)
    v = sampler.sample()
    assert v == 1.0

    # generate 1/2 - 1/2
    values = [0.0, 1.0]
    probs = [0.5, 0.5]
    sampler.update(x=values, p=probs)
    counts = {
        0.0: 0,
        1.0: 0
    }
    for i in range(10000):
        v = sampler.sample()
        counts[v] += 1
    assert counts[0.0] == 5000
    assert counts[1.0] == 5000

    # with array
    sampler = BalancedSampler()
    values = np.array([0.0, 1.0])
    probs = np.array([0.5, 0.5])
    sampler.update(x=values, p=probs)
    counts = {
        0.0: 0,
        1.0: 0
    }
    for i in range(10000):
        v = sampler.sample()
        counts[v] += 1
    assert counts[0.0] == 5000
    assert counts[1.0] == 5000

    # check update
    values2 = [0.0, 1.0]
    probs2 = [0.5, 0.5]
    sampler.update(x=values2, p=probs2)
    assert not sampler._updated
    assert len(sampler._reserved) == 0

    # select
    for i in range(10):
        v = sampler.select(1.0)
        assert v == 1.0
        v = sampler.sample()
        assert v == 0.0
    for i in range(10):
        v = sampler.select(0.0)
        assert v == 0.0
        v = sampler.sample()
        assert v == 1.0

    # not selectable
    v = sampler.select(1.0)
    v = sampler.select(1.0)
    assert v is None

    # exception
    with pytest.raises(RuntimeError):
        v = sampler.select(2.0)

    # generate 1/10 - 9/10
    values2 = [0.0, 1.0]
    probs2 = [0.1, 0.9]
    sampler.update(x=values2, p=probs2)
    assert sampler._updated
    counts = {
        0.0: 0,
        1.0: 0
    }
    for i in range(10000):
        v = sampler.sample()
        counts[v] += 1
    assert counts[0.0] == 1000
    assert counts[1.0] == 9000

    # generate
    values2 = [0.0, 1.0, 2.0]
    probs2 = [0.01, 0.39, 0.60]
    sampler.update(x=values2, p=probs2)
    assert sampler._updated
    counts = {
        0.0: 0,
        1.0: 0,
        2.0: 0,
    }
    for i in range(10000):
        v = sampler.sample()
        counts[v] += 1
    assert counts[0.0] == 100
    assert counts[1.0] == 3900
    assert counts[2.0] == 6000


def test_balanced_sampler_reset():
    sampler = BalancedSampler()

    # generate 1/2 - 1/2
    values = [0.0, 1.0]
    probs = [0.5, 0.5]
    sampler.update(x=values, p=probs)

    # select
    v = sampler.select(1.0)
    assert v == 1.0

    selectable = sampler.selectable(1.0)
    assert selectable is False

    # update does not change the internal states
    sampler.update(x=values, p=probs)
    selectable = sampler.selectable(1.0)
    assert selectable is False

    # reset and update change the internal state
    sampler.reset()
    sampler.update(x=values, p=probs)
    selectable = sampler.selectable(1.0)
    assert selectable is True
