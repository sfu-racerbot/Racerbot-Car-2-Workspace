"""Write every running node's parameters into a run's metadata folder.

Started by record_run.py. Read-only: it only calls each node's
list_parameters and get_parameters services, the same ones `ros2 param dump`
uses, and publishes nothing. It waits `--delay` seconds for the stack to come
up, writes one YAML per node, then keeps watching for `--watch` seconds and
writes any node that appears later. Changes made during the run are in the
bag's /parameter_events.

    ros2 run race_diagnostics snapshot_params --output <run>/metadata/params
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
import yaml

from race_diagnostics.run_metadata import node_file_name, params_tree

POLL_S = 5.0


def _call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def snapshot(node, full_name, out_dir, timeout):
    """Write full_name's parameters; returns the file, or None if it didn't answer."""
    lister = node.create_client(ListParameters, f'{full_name}/list_parameters')
    getter = node.create_client(GetParameters, f'{full_name}/get_parameters')
    try:
        listed = _call(node, lister, ListParameters.Request(), timeout)
        if listed is None:
            return None
        names = list(listed.result.names)
        got = _call(node, getter, GetParameters.Request(names=names), timeout)
        if got is None:
            return None
        values = {n: parameter_value_to_python(v) for n, v in zip(names, got.values)}
    finally:
        node.destroy_client(lister)
        node.destroy_client(getter)
    path = Path(out_dir) / node_file_name(full_name)
    path.write_text(yaml.safe_dump(
        {full_name: {'ros__parameters': params_tree(values)}}, sort_keys=True))
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--output', required=True)
    ap.add_argument('--delay', type=float, default=15.0)
    ap.add_argument('--watch', type=float, default=120.0)
    ap.add_argument('--timeout', type=float, default=2.0)
    args, ros_args = ap.parse_known_args(argv)
    out = Path(args.output).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = Node('snapshot_params')
    done = set()
    try:
        start = time.monotonic()
        while time.monotonic() - start < args.delay:
            rclpy.spin_once(node, timeout_sec=0.2)
        while True:
            for name, namespace in node.get_node_names_and_namespaces():
                full = (namespace.rstrip('/') + '/' + name)
                if full in done or name == node.get_name():
                    continue
                if snapshot(node, full, out, args.timeout) is not None:
                    done.add(full)
            if time.monotonic() - start >= args.delay + args.watch:
                break
            t_poll = time.monotonic()
            while time.monotonic() - t_poll < POLL_S:
                rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(f'parameters of {len(done)} nodes written to {out}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
