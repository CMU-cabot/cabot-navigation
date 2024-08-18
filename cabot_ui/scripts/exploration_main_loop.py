#!/usr/bin/env python3

from rclpy.node import Node
import rclpy

from cabot_ui.explore.test_loop import main as main_loop


class ExplorationMainLoop(Node):
    def __init__(self):
        super().__init__('exploration_main_loop')
        self.logger = self.get_logger()
        self.logger.info("ExplorationMainLoop node started")

        log_dir = self.declare_parameter('log_dir').value
        self.logger.info(f"log_dir: {log_dir}")

        dist_filter = self.declare_parameter('dist_filter').value
        is_sim = self.declare_parameter('is_sim').value

        main_loop(
            dist_filter=dist_filter,
            forbidden_area_filter=False,
            trajectory_filter=False,
            auto=True,
            use_image=False,
            log_dir=log_dir,
            sim=is_sim,
            keyboard=False,
            debug=False
        )


def main(args=None):
    rclpy.init(args=args)

    node = ExplorationMainLoop()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    print(f"Hello from {__file__}")
    main()
