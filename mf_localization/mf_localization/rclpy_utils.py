#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025  IBM Corporation
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

import threading
from typing import Optional

import rclpy
import rclpy.client
from rclpy.impl.rcutils_logger import RcutilsLogger


def call_service(service: rclpy.client.Client, request, timeout_sec: Optional[float] = None,
                 max_retries: int = 1,
                 logger: Optional[RcutilsLogger] = None):
    '''
    call_service function that executes a service call with optional timeout.

    Args:
        service: The service object to be called.
        request: The request to be sent to the service.
        timeout_sec (Optional[float]): Optional timeout in seconds for the service call.
        max_retries: The maximum number of times a service call will be retried.
        logger: rclpy logger.

    Returns:
        The result of the service call, if successful.

    Raises:
        TimeoutError: If a service call timeouts.
        Exception: Exception on future object.
    '''

    if max_retries == 1:
        return _call_service_once(service, request, timeout_sec)
    else:
        exception = RuntimeError("call service fails to catch exception.")
        for i in range(max_retries):
            try:
                return _call_service_once(service, request, timeout_sec)
            except (TimeoutError, Exception) as e:
                if logger is not None:
                    logger.info(F"failed to call {service.srv_name}. retries={i}")
                exception = e

        raise exception


def _call_service_once(service: rclpy.client.Client, request, timeout_sec: Optional[float] = None):
    '''
    call_service_once function that executes a service call with optional timeout.

    Args:
        service: The service object to be called.
        request: The request to be sent to the service.
        timeout_sec (Optional[float]): Optional timeout in seconds for the service call.

    Returns:
        The result of the service call, if successful.

    Raises:
        TimeoutError: If a service call timeouts.
        Exception: Exception on future object.
    '''

    # service call with timeout
    event = threading.Event()

    def unblock(future) -> None:
        nonlocal event
        event.set()

    future = service.call_async(request)
    future.add_done_callback(unblock)

    if not future.done():
        if not event.wait(timeout_sec):
            service.remove_pending_request(future)
            raise TimeoutError(F"{service.srv_name} service call timeout")

    exception = future.exception()
    if exception is not None:
        raise exception
    return future.result()
