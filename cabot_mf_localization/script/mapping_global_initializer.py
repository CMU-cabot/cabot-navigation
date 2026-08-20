#!/usr/bin/env python3

# Copyright (c) 2026  Carnegie Mellon University
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

"""Pause bag playback until an unseeded trajectory is globally connected."""

import argparse
import sys
import time

import rclpy
from cartographer_ros_msgs.srv import ReadMetrics
from rosbag2_interfaces.srv import Pause
from rosbag2_interfaces.srv import Resume


GLOBAL_CONSTRAINTS_FAMILY = (
    "mapping_constraints_constraint_builder_2d_constraints"
)
CONSTRAINT_QUEUE_FAMILY = (
    "mapping_constraints_constraint_builder_2d_queue_length"
)
WORK_QUEUE_FAMILY = "mapping_2d_pose_graph_work_queue_size"
POSE_GRAPH_CONSTRAINTS_FAMILY = "mapping_2d_pose_graph_constraints"


def call(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"service call to {client.srv_name} timed out")
    return future.result()


def metric_value(response, family_name, expected_labels=None):
    expected_labels = expected_labels or {}
    for family in response.metric_families:
        if family.name != family_name:
            continue
        for metric in family.metrics:
            labels = {label.key: label.value for label in metric.labels}
            if all(
                labels.get(key) == value
                for key, value in expected_labels.items()
            ):
                return metric.value
    return None


def read_progress(node, metrics_client, service_timeout):
    response = call(
        node, metrics_client, ReadMetrics.Request(), service_timeout
    )
    if response.status.code != 0:
        raise RuntimeError(
            f"Cartographer metrics are unavailable: {response.status.message}"
        )

    found = metric_value(
        response,
        GLOBAL_CONSTRAINTS_FAMILY,
        {"search_region": "global", "matcher": "found"},
    )
    inter_trajectory_constraints = metric_value(
        response,
        POSE_GRAPH_CONSTRAINTS_FAMILY,
        {"tag": "inter_submap", "trajectory": "different"},
    )
    constraint_queue = metric_value(response, CONSTRAINT_QUEUE_FAMILY)
    work_queue = metric_value(response, WORK_QUEUE_FAMILY)
    if (
        found is None
        or inter_trajectory_constraints is None
        or constraint_queue is None
        or work_queue is None
    ):
        raise RuntimeError("required Cartographer metrics were not registered")
    return (
        int(found),
        int(inter_trajectory_constraints),
        int(constraint_queue),
        int(work_queue),
    )


def wait_for_service(client, deadline):
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            return
    raise RuntimeError(f"service {client.srv_name} did not become available")


def sleep_until(node, target_time, deadline):
    while time.monotonic() < target_time:
        if time.monotonic() >= deadline:
            raise RuntimeError("global initialization timed out during playback")
        rclpy.spin_once(
            node,
            timeout_sec=min(0.2, target_time - time.monotonic()),
        )


def wait_for_queue_state(
    node,
    metrics_client,
    service_timeout,
    pending_constraint_wait,
    inter_trajectory_baseline,
    deadline,
):
    last_progress = None
    work_queue_empty_since = None
    while time.monotonic() < deadline:
        progress = read_progress(node, metrics_client, service_timeout)
        if progress != last_progress:
            (
                found,
                inter_trajectory_constraints,
                constraint_queue,
                work_queue,
            ) = progress
            print(
                "global constraints found="
                f"{found}, inter-trajectory constraints="
                f"{inter_trajectory_constraints}, "
                f"constraint queue={constraint_queue}, "
                f"pose graph work queue={work_queue}",
                flush=True,
            )
            last_progress = progress

        (
            found,
            inter_trajectory_constraints,
            constraint_queue,
            work_queue,
        ) = progress
        if found == 0:
            # Loaded states can already contain constraints between their
            # trajectories.  Keep tracking that baseline until the new
            # trajectory produces its first global match.
            inter_trajectory_baseline = max(
                inter_trajectory_baseline, inter_trajectory_constraints
            )

        connected = (
            found > 0
            and inter_trajectory_constraints > inter_trajectory_baseline
        )
        if connected and work_queue == 0:
            return found, True, inter_trajectory_baseline
        if found == 0 and constraint_queue == 0 and work_queue == 0:
            return found, False, inter_trajectory_baseline

        if work_queue == 0:
            if work_queue_empty_since is None:
                work_queue_empty_since = time.monotonic()
            elif (
                time.monotonic() - work_queue_empty_since
                >= pending_constraint_wait
            ):
                # ConstraintBuilder only clears its result queue from a pose
                # graph optimization callback.  With a deliberately sparse
                # optimization interval, a short initial segment may find a
                # global match without reaching that interval.  Let playback
                # collect the nodes needed to trigger the callback.
                return found, False, inter_trajectory_baseline
        else:
            work_queue_empty_since = None
        sleep_until(node, time.monotonic() + 1.0, deadline)
    raise RuntimeError("global initialization timed out while draining work")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--play-seconds", type=float, default=30.0)
    parser.add_argument("--startup-delay", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument(
        "--pending-constraint-wait", type=float, default=60.0
    )
    args = parser.parse_args()

    if args.play_seconds <= 0.0:
        parser.error("--play-seconds must be positive")
    if args.startup_delay < 0.0:
        parser.error("--startup-delay must be non-negative")
    if not 0.0 < args.rate <= 1.0:
        parser.error("--rate must be greater than zero and no greater than one")
    if (
        args.timeout <= 0.0
        or args.service_timeout <= 0.0
        or args.pending_constraint_wait <= 0.0
    ):
        parser.error("timeouts must be positive")

    rclpy.init()
    node = rclpy.create_node("mapping_global_initializer")
    pause_client = node.create_client(Pause, "/rosbag2_player/pause")
    resume_client = node.create_client(Resume, "/rosbag2_player/resume")
    metrics_client = node.create_client(ReadMetrics, "/read_metrics")
    deadline = time.monotonic() + args.timeout

    try:
        for client in (pause_client, resume_client, metrics_client):
            wait_for_service(client, deadline)

        initial_progress = read_progress(
            node, metrics_client, args.service_timeout
        )
        inter_trajectory_baseline = initial_progress[1]

        cycle = 0
        while True:
            cycle += 1
            wall_play_seconds = args.play_seconds / args.rate
            if cycle == 1:
                wall_play_seconds += args.startup_delay
            print(
                f"global initialization cycle {cycle}: playing "
                f"{args.play_seconds:g} seconds of bag data",
                flush=True,
            )
            sleep_until(
                node, time.monotonic() + wall_play_seconds, deadline
            )
            call(node, pause_client, Pause.Request(), args.service_timeout)
            print(
                "bag playback paused; waiting for global constraint work",
                flush=True,
            )

            (
                found,
                connected,
                inter_trajectory_baseline,
            ) = wait_for_queue_state(
                node,
                metrics_client,
                args.service_timeout,
                args.pending_constraint_wait,
                inter_trajectory_baseline,
                deadline,
            )
            if not connected:
                if found > 0:
                    reason = (
                        "global constraint found but awaiting a pose graph "
                        "optimization; collecting another"
                    )
                else:
                    reason = "no global constraint found; collecting another"
                print(
                    f"{reason} {args.play_seconds:g} seconds",
                    flush=True,
                )
                call(
                    node,
                    resume_client,
                    Resume.Request(),
                    args.service_timeout,
                )
                continue

            call(
                node,
                resume_client,
                Resume.Request(),
                args.service_timeout,
            )
            print(
                "global initialization complete; resuming bag playback",
                flush=True,
            )
            return 0

        raise RuntimeError("global initialization timed out")
    except RuntimeError as error:
        print(f"mapping_global_initializer.py: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
