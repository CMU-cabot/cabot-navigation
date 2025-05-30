# Copyright (c) 2020, 2022  Carnegie Mellon University
# Copyright (c) 2024  ALPS ALPINE CO., LTD.
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

"""
Turn Detector Module
"""

import math
import enum
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui import geoutil

smoothParam = 0.015
lookAheadStepsA = int(2 / 0.02)
thtAngle0 = math.radians(10)
thtAngle1 = math.radians(1)
thtMinimumTurn = math.radians(60)
thtMinimumTurnDev = math.radians(0.01)
# thtMinimumDev = math.radians(20) / 57.32  # 57.32 is a magic number
thtMinimumDev = math.radians(0.3)
bandWidth = 1.0
lookAheadStepsB = int(0.5 / 0.02)


class Turn:
    class Type(enum.Enum):
        Normal = 1
        Avoiding = 2

    def __init__(self, pose, angle, turn_type=Type.Normal, end=None):
        self.pose = pose
        self.start = pose.pose.position
        self.angle = angle
        self.turn_type = turn_type
        self.end = end
        self.text = ""

        if turn_type == Turn.Type.Normal:
            if angle < -math.pi / 3:
                self.text = F"Turn Right ({angle:.2f})"
            elif angle > math.pi / 3:
                self.text = F"Turn Left ({angle:.2f})"
        elif turn_type == Turn.Type.Avoiding:
            if angle < 0:
                self.text = F"Dev Right ({angle:.2f})"
            elif angle > 0:
                self.text = F"Dev Left ({angle:.2f})"
        else:
            self.text = F"Unknown ({angle:.2f})"

        self.passed_directional_indicator = False
        self.passed_vibrator = False

    def __str__(self):
        return F"<Turn at ({self.start.x}, {self.start.y}), {self.angle}, type({self.turn_type})>"

    def __repr__(self):
        return self.__str__()

    def distance_to(self, other):
        if isinstance(other, Turn):
            return np.sqrt((self.start.x - other.start.x) ** 2 + (self.start.y - other.start.y) ** 2)
        elif isinstance(other, geoutil.Pose):
            pose = geoutil.Pose(pose_msg=self.pose)
            pose.r = pose.r + math.pi
            dist_TR = other.distance_to(pose)
            pose_TR = geoutil.Pose.pose_from_points(pose, other)
            yaw = geoutil.diff_angle(pose.orientation, pose_TR.orientation)
            adjusted = dist_TR * math.cos(yaw)
            CaBotRclpyUtil.debug(f"dist={dist_TR}, yaw={yaw}, adjusted={adjusted}")
            return adjusted


class TurnDetector:
    def __init__(self):
        pass

    @staticmethod
    def detects(path, current_pose=None, visualize=False):
        start = 0

        # requires more than or eqauls to 13 poses
        if len(path.poses) < 13:
            return []

        length = len(path.poses)
        x = np.array([pose.pose.position.x for pose in path.poses])
        y = np.array([pose.pose.position.y for pose in path.poses])
        dx = np.zeros(length)
        dy = np.zeros(length)
        yaw = np.zeros(length)
        dyaw = np.zeros(length)
        dyaw2 = np.zeros(length)
        TurnStarts, TurnEnds, Angles, Types = [], [], [], []

        if current_pose is not None:
            dx = x - current_pose.x
            dy = y - current_pose.y
            distances = np.hypot(dx, dy)
            min_i = int(np.argmin(distances))
        start = min_i

        # get differential
        dx[1:-1] = x[2:] - x[:-2]
        dy[1:-1] = y[2:] - y[:-2]
        dx[0] = x[1] - x[0]
        dy[0] = y[1] - y[0]
        dx[-1] = x[-1] - x[-2]
        dy[-1] = y[-1] - y[-2]

        # Replace sequential yaw calculations with a vectorized version:
        yaw = np.unwrap(np.arctan2(dy, dx))

        # lowpass filtering
        B, A = signal.iirfilter(3, smoothParam, btype='lowpass')
        yaw, yawRaw = signal.filtfilt(B, A, yaw), yaw

        # Calculate first-order derivatives of yaw
        dyaw[0:-1] = yaw[1:] - yaw[:-1]
        N = 10
        for i in range(0, N):
            dyaw2[0:-N] += abs(dyaw[i:-N + i]) / N

        # find turns
        i = start
        t_i = 0
        while (i < length - 1):
            j = int(min(length - 1, i + lookAheadStepsA))

            # if angle difference is bigger than the threshold
            if abs(angDiff(yaw[i], yaw[j])) < thtAngle0:
                # print([i, abs(angDiff(yaw[i],yaw[j]))])
                i += 1
                continue

            else:  # Found a turnStart
                if angDiff(yaw[i], yaw[j]) > 0:  # Right Turn, angle decrease
                    RightTurn = 1
                else:
                    RightTurn = -1

                # Search turnStart - Fine search
                k = j

                # find point that turning starts
                while k > i and dyaw2[k] > thtMinimumTurnDev:
                    k -= 1
                TurnStarts.append(k)

                # Search turnEnd - Fine search
                k = j

                # find point that turning ends
                while k < length - 1 and dyaw2[k] > thtMinimumTurnDev:
                    k += 1
                TurnEnds.append(k)

                # smaller than minimum turn
                if abs(yaw[TurnEnds[-1]] - yaw[TurnStarts[-1]]) < thtMinimumTurn:
                    # ignore if smaller than minimum deviation
                    if dyaw2[TurnStarts[-1]:TurnEnds[-1]].size > 0 and max(dyaw2[TurnStarts[-1]:TurnEnds[-1]]) < thtMinimumDev:
                        del TurnStarts[-1]
                        del TurnEnds[-1]
                    else:
                        diff = max(yaw[TurnStarts[-1]:TurnEnds[-1]]) - min(yaw[TurnStarts[-1]:TurnEnds[-1]])
                        Angles.append(-RightTurn * diff)
                        # Angles.append(-RightTurn * math.pi / 7)
                        Types.append(Turn.Type.Avoiding)
                else:
                    if TurnEnds[t_i] < TurnStarts[t_i]:
                        TurnEnds[t_i], TurnStarts[t_i] = TurnStarts[t_i], TurnEnds[t_i]
                    Angles.append(yaw[TurnEnds[t_i]] - yaw[TurnStarts[t_i]])
                    Types.append(Turn.Type.Normal)
                    t_i += 1
                i = k

        CaBotRclpyUtil.debug("*******")
        CaBotRclpyUtil.debug(f"{TurnStarts}")
        CaBotRclpyUtil.debug(f"{TurnEnds}")
        CaBotRclpyUtil.debug(f"{Angles}")
        CaBotRclpyUtil.debug(f"{Types}")

        turns = []
        for i, j, angle, turn_type in zip(TurnStarts, TurnEnds, Angles, Types):
            sp = path.poses[i]
            sp.header.frame_id = path.header.frame_id
            ep = path.poses[j]
            ep.header.frame_id = path.header.frame_id
            turns.append(Turn(sp, math.degrees(angle), turn_type, ep))

        for turn in turns:
            CaBotRclpyUtil.debug(turn.text)

        if visualize:
            TurnDetector._visualize(yaw, x, y, TurnStarts, TurnEnds, yawRaw, yaw, dyaw, dyaw2)

        return turns

    @staticmethod
    def _visualize(yaw, x, y, TurnStarts, TurnEnds, yawRaw, yawLP, dyaw, dyaw2):
        x_stp = np.array(range(len(yaw))) * 0.02

        if 1:  # plot map
            plt.figure(1)
            plt.plot(x, y, 'b-', label='path - top view')
            plt.plot(x[TurnStarts], y[TurnStarts], 'ro', label='TurnStarts')
            plt.plot(x[TurnEnds], y[TurnEnds], 'go', label='TurnEnds')
            # plt.plot(x[TurnB],y[TurnB],'*',label='TurnEnds')
            # plt.legend()
            plt.ylabel('Y (meter)')
            plt.xlabel('X (meter)')

        if 1:  # plot angle theta(degree)
            plt.figure(2)
            # plt.subplot(3,1,1)
            plt.plot(x_stp, yawRaw, '.-', label='yaw')
            plt.plot(x_stp, yawLP, '.-', label='Lowpass yaw')
            plt.plot(x_stp[TurnStarts], yawLP[TurnStarts], 'ro', label='TurnStarts')
            plt.plot(x_stp[TurnEnds], yawLP[TurnEnds], 'go', label='TurnEnds')
            plt.ylabel('orientation (degree)')
            plt.xlabel('Distance(meter)')
            plt.legend()
            # plt.subplot(3,1,2)
            # plt.plot(x_stp,dyaw,'.-')
            # plt.subplot(3,1,3)
            # plt.plot(x_stp,ddyaw,'.-')

        if 1:
            plt.figure(3)
            plt.plot(dyaw)
            plt.plot(TurnStarts, dyaw[TurnStarts], 'ro', label='TurnStarts')
            plt.plot(TurnEnds, dyaw[TurnEnds], 'go', label='TurnStarts')
            plt.plot(dyaw2)
            plt.plot(TurnStarts, dyaw2[TurnStarts], 'rx', label='TurnStarts2')
            plt.plot(TurnEnds, dyaw2[TurnEnds], 'gx', label='TurnStarts2')

        plt.show()

        if 0:  # plot yaw in frequency domain
            yawFAmp = np.abs(np.fft.fft(yaw))
            wn = 1.0 * np.array(range(len(yaw))) / len(yaw)
            plt.plot(wn, yawFAmp, '.', label='.')
            plt.ylabel('amp')
            plt.xlabel('f(in unkown unit)')
            plt.legend()
            plt.show()


def xy2angle(dx, dy):
    return math.atan2(dy, dx)


def getDist(x0, y0, x1, y1):
    return np.hypot(x1 - x0, y1 - y0)


def angDiff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi
