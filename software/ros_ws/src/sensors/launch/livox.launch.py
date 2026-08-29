# ============================================================= #
#                                  ____                         #
#                                 / . .\                        #
#                                 \  ---<                       #
#                                  \  /                         #
#                        __________/ /                          #
#                     -=:___________/                           #
# ------------------------------------------------------------- #
#                         Livox runner                          #
#               [ Ask Dan :) if something breaks ]              #
# ------------------------------------------------------------- #
#                                                               #
# This launch file launches the livox driver.                   #
# It publishes a 3D pointcloud in the livox_frame.              #
# It uses the config file found in config/livox_config.json.    #
#                                                               #
# It creates a tmp file with the host IP.                       #
# The Livox IP should be 192.168.1.21, if not network is weird. #
#                                                               #
# The host IP is picked from the interface by matching the      #
# lidar's subnet, so an interface carrying several addresses    #
# (e.g. a DHCP lease plus a static on the lidar subnet) works.  #
# Override with host_ip:=<addr> if the guess is ever wrong.     #
#                                                               #
# ============================================================= #
# ROS Things
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Config Handling
import ipaddress
import json
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import subprocess


def get_host_addresses(interface: str):
    """
    Returns every IPv4 address (with prefix) assigned to the interface.
    """

    result = subprocess.run(
        ["ip", "-4", "addr", "show", interface],
        capture_output=True,
        text=True,
        check=True,
    )

    addresses = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.startswith("inet "):
            addresses.append(ipaddress.ip_interface(line.split()[1]))

    if not addresses:
        raise RuntimeError(f"No IPv4 address found on {interface}")

    return addresses


def select_host_ip(interface: str, lidar_ips: list, override: str):
    """
    Picks the address on `interface` that can actually reach the lidars.

    The driver hands this IP to the lidar as the destination for point data,
    so an address on the wrong subnet makes the driver log a successful init
    and then sit there receiving nothing, forever. Fail loudly instead.
    """

    if override:
        return override

    addresses = get_host_addresses(interface)

    for address in addresses:
        if all(ip in address.network for ip in lidar_ips):
            return str(address.ip)

    raise RuntimeError(
        f"No address on {interface} shares a subnet with the configured "
        f"lidar(s) {[str(ip) for ip in lidar_ips]}.\n"
        f"  {interface} has: {[str(a) for a in addresses]}\n"
        f"Add an address on the lidar subnet, e.g.\n"
        f"  sudo ip addr add {lidar_ips[0].exploded.rsplit('.', 1)[0]}.50/24 "
        f"dev {interface}\n"
        f"or pass host_ip:=<addr> to override this check."
    )


def create_livox_config(default_config_path: str, interface: str, host_ip_arg: str):
    """
    Creates a temporary file with correct host IP that the driver uses.
    """

    config_path = Path(default_config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    # Load default config
    with open(config_path, "r") as f:
        config = json.load(f)

    lidar_ips = [
        ipaddress.ip_address(lidar["ip"]) for lidar in config.get("lidar_configs", [])
    ]
    if not lidar_ips:
        raise RuntimeError(f"No lidar_configs entries in {config_path}")

    host_ip = select_host_ip(interface, lidar_ips, host_ip_arg)

    # Insert HOST IP into config
    if "MID360" in config and "host_net_info" in config["MID360"]:
        for key in config["MID360"]["host_net_info"]:
            if key.endswith("_ip"):
                config["MID360"]["host_net_info"][key] = host_ip

    # Create temp config file
    with tempfile.NamedTemporaryFile(
        mode="wt",
        dir="/tmp",
        delete=False,
        prefix="livox.",
    ) as temp:
        json.dump(config, temp, indent=2)

        print("=========================")
        print("|   TEMP FILE CREATED   |")
        print("=========================")
        print(f"Located: {temp.name}")
        print(f"Interface: {interface}")
        print(f"Host IP: {host_ip}")
        print(f"Lidar IP(s): {', '.join(str(ip) for ip in lidar_ips)}")
        print("=========================")

        return temp.name


def launch_setup(context, *args, **kwargs):
    """
    Runtime launch setup (required to resolve LaunchConfiguration)
    """

    interface = LaunchConfiguration("interface").perform(context)
    host_ip_arg = LaunchConfiguration("host_ip").perform(context)

    config_path = (
        Path(get_package_share_directory("sensors")) / "config" / "livox_config.json"
    )

    livox_path = create_livox_config(str(config_path), interface, host_ip_arg)

    return [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            parameters=[
                {
                    "xfer_format": 0,
                    "multi_topic": 0,
                    "data_src": 0,
                    "publish_freq": 10.0,
                    "output_data_type": 0,
                    "frame_id": "livox_frame",
                    "user_config_path": livox_path,
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interface",
                default_value="eth1",
                description="Network interface to get host IP from",
            ),
            DeclareLaunchArgument(
                "host_ip",
                default_value="",
                description="Override the host IP given to the lidar "
                "(default: the address on the interface matching the lidar subnet)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
