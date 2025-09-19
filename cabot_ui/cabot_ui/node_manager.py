import threading
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor, ExternalShutdownException


class NodeManager:
    def __init__(self):
        self._nodes = {}
        self._threads = []

    def get_node(self, name: str = None, single_threaded: bool = False):
        node_name = f"cabot_ui_manager_{name}" if name is not None else "cabot_ui_manager"
        key = name if name is not None else "default"
        if key in self._nodes:
            return self._nodes[key]
        node = Node(node_name, start_parameter_services=False)
        executor = SingleThreadedExecutor() if single_threaded else MultiThreadedExecutor()
        thread = threading.Thread(target=self._run_node(node, executor, name))
        thread.start()
        self._nodes[key] = node
        self._threads.append(thread)
        return node

    def _run_node(self, target_node, executor, name):
        def _run_node():
            # debug code to analyze the bottle neck of nodes
            # high frequency spinning node should have smaller number of waits
            #
            # import time
            # count = 0
            # start = time.time()
            executor.add_node(target_node)
            try:
                while rclpy.ok():
                    # count += 1
                    # target_node.get_logger().info(f"spin rate {name} {count / (time.time()-start):.2f}Hz - \n"
                    #                               f"  subscriptions {[sub.topic_name for sub in list(target_node.subscriptions)]}\n"
                    #                               f"  timers {len(list(target_node.timers))}\n"
                    #                               f"  clients {[cli.srv_name for cli in list(target_node.clients)]}\n"
                    #                               f"  services {len(list(target_node.services))}\n"
                    #                               f"  guards {len(list(target_node.guards))}\n"
                    #                               f"  waitables {len(list(target_node.waitables))}\n",
                    #                               throttle_duration_sec=1.0)
                    executor.spin_once()
            except KeyboardInterrupt:
                target_node.get_logger().info(f"Shutting down {name} node")
            except ExternalShutdownException:
                pass
            except:  # noqa: 722
                target_node.get_logger().error(traceback.format_exc())
            target_node.destroy_node()
            target_node.get_logger().info(f"{name} node terminated")
        return _run_node

    def join(self):
        for thread in self._threads:
            thread.join()


"""
        node = Node('cabot_ui_manager', start_parameter_services=False)
        nav_node = Node("cabot_ui_manager_navigation", start_parameter_services=False)
        tf_node = Node("cabot_ui_manager_tf", start_parameter_services=False)
        srv_node = Node("cabot_ui_manager_navigation_service", start_parameter_services=False)
        act_node = Node("cabot_ui_manager_navigation_actions", start_parameter_services=False)
        soc_node = Node("cabot_ui_manager_navigation_social", start_parameter_services=False)
        desc_node = Node("cabot_ui_manager_description", start_parameter_services=False)
        nodes = [node, nav_node, tf_node, srv_node, act_node, soc_node, desc_node]
        executors = [MultiThreadedExecutor(),
                        MultiThreadedExecutor(),
                        SingleThreadedExecutor(),
                        SingleThreadedExecutor(),
                        SingleThreadedExecutor(),
                        SingleThreadedExecutor(),
                        SingleThreadedExecutor(),
                        ]
        names = ["node", "tf", "nav", "srv", "act", "soc", "desc"]
        manager = CabotUIManager(node, nav_node, tf_node, srv_node, act_node, soc_node, desc_node)

        threads = []
        for tnode, executor, name in zip(nodes, executors, names):
            def run_node(target_node, executor, name):
                def _run_node():
                    # debug code to analyze the bottle neck of nodes
                    # high frequency spinning node should have smaller number of waits
                    #
                    # import time
                    # count = 0
                    # start = time.time()
                    executor.add_node(target_node)
                    try:
                        while rclpy.ok():
                            # count += 1
                            # target_node.get_logger().info(f"spin rate {name} {count / (time.time()-start):.2f}Hz - \n"
                            #                               f"  subscriptions {[sub.topic_name for sub in list(target_node.subscriptions)]}\n"
                            #                               f"  timers {len(list(target_node.timers))}\n"
                            #                               f"  clients {[cli.srv_name for cli in list(target_node.clients)]}\n"
                            #                               f"  services {len(list(target_node.services))}\n"
                            #                               f"  guards {len(list(target_node.guards))}\n"
                            #                               f"  waitables {len(list(target_node.waitables))}\n",
                            #                               throttle_duration_sec=1.0)
                            executor.spin_once()
                    except KeyboardInterrupt:
                        target_node.get_logger().info(f"Shutting down {name} node")
                    except:  # noqa: 722
                        target_node.get_logger().error(traceback.format_exc())
                    target_node.destroy_node()
                return _run_node

            thread = threading.Thread(target=run_node(tnode, executor, name))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()        
"""