#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cabot_ui.explore.test_image import main
import argparse
import os
import logging

if __name__ == '__main__':

    main(
        no_explain_mode=args.no_explain, 
        log_dir=args.log_dir, 
        once=args.once, 
        is_sim=args.sim,
        semantic_map_mode=args.semantic_map,
        intersection_detection_mode=args.intersection_detection,
        surronding_explain_mode=args.surronding_explain,
        should_speak=args.speak
    )