# Copyright (c) 2025  Carnegie Mellon University
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

import copy
import traceback

from rcl_interfaces.srv import SetParameters, GetParameters
import rcl_interfaces.msg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter


class ParamManager:
    def __init__(self, node):
        self.node = node
        self.clients = {}
        self.callback_group = MutuallyExclusiveCallbackGroup()

    def get_client(self, node_name, service_type, service_name):
        key = f'{node_name}/{service_name}'
        if key not in self.clients:
            for i in range(10):
                client = self.node.create_client(service_type, key, callback_group=self.callback_group)
                if client.wait_for_service(timeout_sec=1.0):
                    self.node.get_logger().info(f'{key} is available')
                    self.clients[key] = client
                    break
                self.node.get_logger().error(f'{key} is not available (retry {i+1})...')
        if key in self.clients:
            return self.clients[key]
        return None

    def change_parameter(self, node_name, param_dict, callback):
        def done_callback(future):
            callback(node_name, future)
        request = SetParameters.Request()
        for param_name, param_value in param_dict.items():
            new_parameter = Parameter(param_name, value=param_value)
            request.parameters.append(new_parameter.to_parameter_msg())
        client = self.get_client(node_name, SetParameters, "set_parameters")
        if client:
            future = client.call_async(request)
            future.add_done_callback(done_callback)
        else:
            done_callback(None)

    def change_parameters(self, params, callback):
        params = copy.deepcopy(params)

        def sub_callback(node_name, future):
            del params[node_name]
            self.node.get_logger().info(f"change_parameter sub_callback {node_name} {len(params)} {future.result() if future else None}")
            if len(params) == 0:
                if future:
                    callback(future.result())
                else:
                    callback(None)
            else:
                self.change_parameters(params, callback)
        for node_name, param_dict in params.items():
            self.node.get_logger().info(f"call change_parameter {node_name}, {param_dict}")
            try:
                self.change_parameter(node_name, param_dict, sub_callback)
            except:  # noqa: 722
                self.node.get_logger().error(traceback.format_exc())
            break

    def request_parameter(self, node_name, param_list, callback):
        def done_callback(future):
            callback(node_name, param_list, future)
        request = GetParameters.Request()
        request.names = param_list
        client = self.get_client(node_name, GetParameters, "get_parameters")
        if client:
            future = client.call_async(request)
            future.add_done_callback(done_callback)
        else:
            done_callback(None)

    def request_parameters(self, params, callback):
        self.rcount = 0
        self.result = {}

        def sub_callback(node_name, param_list, future):
            self.rcount += 1
            self.result[node_name] = {}
            if future:
                for name, value in zip(param_list, future.result().values):
                    msg = rcl_interfaces.msg.Parameter()
                    msg.name = name
                    msg.value = value
                    param = Parameter.from_parameter_msg(msg)
                    self.result[node_name][name] = param.value
            if self.rcount == len(params):
                if future:
                    self.node.get_logger().info(f"request_parameter sub_callback {self.rcount} {len(params)} {node_name}, {param_list}, {future.result()}")
                else:
                    self.node.get_logger().info(f"request_parameter sub_callback {self.rcount} {len(params)} {node_name}, {param_list}, None")
                callback(self.result)
        for node_name, param_list in params.items():
            self.node.get_logger().info(f"call request_parameter {node_name}, {param_list}")
            try:
                self.request_parameter(node_name, param_list, sub_callback)
            except:  # noqa: 722
                self.node.get_logger().error(traceback.format_exc())
