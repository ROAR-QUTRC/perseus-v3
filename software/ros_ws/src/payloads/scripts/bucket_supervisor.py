#!/usr/bin/env python3
"""Brings the bucket's read-only side back after a fault.

When BucketHardware's read() fails (a stale encoder or a bank fault while a
controller is commanding the bucket), controller_manager deactivates the
BucketSystem component and every controller tied to it. The drive is untouched,
but the bucket's encoders stop reaching /joint_states until someone re-activates
them. This does that, once a second:

  * re-activates the BucketSystem hardware component, and
  * re-activates bucket_joint_state_broadcaster and bucket_linkage_broadcaster.

Both are read-only, so this is always safe. It never re-arms the command
controller on its own - after a fault, or after the operator took the bucket with
the gamepad, a person decides when autonomy gets it back. For that:

    ros2 run payloads bucket_supervisor.py --rearm

which recovers from a fault as above and then hands the bucket back to
bucket_trajectory_controller. If the operator has the bucket
(/bucket/operator_override), that goes through the teleop driver's /bucket/rearm
service, so the driver stops sending speeds in the same step; it is refused while
a stick is still held. (`bucket_cli rearm` does the same.)

Re-activating the hardware does not block on the encoders
(BucketHardware::on_activate), so this never stalls the drive's control loop;
the trajectory controller is refused until the encoders are current.
"""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

HARDWARE = "BucketSystem"
BROADCASTERS = ("bucket_joint_state_broadcaster", "bucket_linkage_broadcaster")
TRAJECTORY = "bucket_trajectory_controller"
CALL_TIMEOUT_S = 2.0


class BucketSupervisor(Node):
    def __init__(self, controller_manager: str):
        super().__init__("bucket_supervisor")
        cm = controller_manager.rstrip("/")
        self._list_hardware = self.create_client(
            ListHardwareComponents, f"{cm}/list_hardware_components"
        )
        self._set_hardware = self.create_client(
            SetHardwareComponentState, f"{cm}/set_hardware_component_state"
        )
        self._list_controllers = self.create_client(
            ListControllers, f"{cm}/list_controllers"
        )
        self._switch = self.create_client(SwitchController, f"{cm}/switch_controller")
        self._rearm = self.create_client(Trigger, "/bucket/rearm")
        self._override = False
        # Set once this has brought the hardware back, until its controllers are
        # back too. Outside of a recovery the controllers are left alone, so this
        # never races the launch file's spawners at startup.
        self._recovering = False
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, "/bucket/operator_override", self._on_override, latched
        )

    def _on_override(self, msg: Bool):
        self._override = msg.data

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_S)
        return future.result() if future.done() else None

    def check(self, with_trajectory: bool, force: bool = False) -> bool:
        """One pass. Returns False when controller_manager can't be reached, or
        when with_trajectory is asked for and the bucket could not be re-armed.

        force restores the controllers even when this pass didn't recover the
        hardware (--rearm / --once, run by a person).
        """
        hardware = self._call(self._list_hardware, ListHardwareComponents.Request())
        if hardware is None:
            self.get_logger().error(
                "controller_manager is not reachable", throttle_duration_sec=10.0
            )
            return False
        component = next((c for c in hardware.component if c.name == HARDWARE), None)
        if component is None:
            return True  # no bucket on this rover
        if component.state.id != State.PRIMARY_STATE_ACTIVE:
            request = SetHardwareComponentState.Request()
            request.name = HARDWARE
            request.target_state = State(id=State.PRIMARY_STATE_ACTIVE, label="active")
            response = self._call(self._set_hardware, request)
            if response is None or not response.ok:
                self.get_logger().error(
                    f"Could not re-activate {HARDWARE} (was {component.state.label})",
                    throttle_duration_sec=10.0,
                )
                return True
            self.get_logger().warn(
                f"Re-activated {HARDWARE} (was {component.state.label})"
            )
            self._recovering = True

        if not (self._recovering or force):
            return True

        # With the operator in charge, the trajectory controller comes back through
        # the teleop driver instead (below), so it stops sending speeds at once.
        wanted = list(BROADCASTERS)
        if with_trajectory and not self._override:
            wanted.append(TRAJECTORY)

        controllers = self._call(self._list_controllers, ListControllers.Request())
        if controllers is None:
            return False
        inactive = [
            c.name
            for c in controllers.controller
            if c.name in wanted and c.state == "inactive"
        ]
        restored = True
        activated = set()
        for name in inactive:
            # One at a time, STRICT, so one that can't come up (the trajectory
            # controller with stale encoders) doesn't hold up the others.
            request = SwitchController.Request()
            request.activate_controllers = [name]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            response = self._call(self._switch, request)
            if response is not None and response.ok:
                activated.add(name)
                self.get_logger().warn(f"Re-activated {name}")
            else:
                restored = False
                self.get_logger().error(
                    f"Could not re-activate {name}", throttle_duration_sec=10.0
                )
        if restored:
            self._recovering = False

        if with_trajectory:
            if self._override:
                return self._rearm_from_operator()
            active = TRAJECTORY in activated or any(
                c.name == TRAJECTORY and c.state == "active"
                for c in controllers.controller
            )
            if active:
                self.get_logger().info(f"{TRAJECTORY} has the bucket")
            else:
                self.get_logger().error(
                    f"{TRAJECTORY} is not active - is it loaded, and are the "
                    "encoders current? See the controller_manager log"
                )
            return active
        return True

    def _rearm_from_operator(self) -> bool:
        """Hands the bucket back from the gamepad via bucket_driver's /bucket/rearm."""
        response = self._call(self._rearm, Trigger.Request())
        if response is None:
            self.get_logger().error(
                "The operator has the bucket, but /bucket/rearm is not answering - "
                "is bucket_driver (the bucket teleop) running?"
            )
            return False
        if response.success:
            self.get_logger().info(f"Re-armed from the operator: {response.message}")
        else:
            self.get_logger().error(f"Re-arm refused: {response.message}")
        return response.success


def main():
    rclpy.init(args=sys.argv)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--rearm",
        action="store_true",
        help=f"recover from a fault and hand the bucket back to {TRAJECTORY}, "
        "from a fault or from the operator, then exit (same as --once --with-trajectory)",
    )
    parser.add_argument("--once", action="store_true", help="one pass, then exit")
    parser.add_argument(
        "--with-trajectory",
        action="store_true",
        help=f"also hand the bucket back to {TRAJECTORY}",
    )
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--period", type=float, default=1.0, help="seconds per pass")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if args.rearm:
        args.once = args.with_trajectory = True

    node = BucketSupervisor(args.controller_manager)
    try:
        # Let the latched override flag arrive before the first decision.
        rclpy.spin_once(node, timeout_sec=0.5)
        if args.once:
            ok = node.check(args.with_trajectory, force=True)
            sys.exit(0 if ok else 1)
        while rclpy.ok():
            node.check(args.with_trajectory)
            deadline = time.monotonic() + args.period
            while rclpy.ok() and (remaining := deadline - time.monotonic()) > 0:
                rclpy.spin_once(node, timeout_sec=remaining)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
