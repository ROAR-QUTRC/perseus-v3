# vcan-setup.sh

## Motivation

Many of the ROS2 nodes used on perseus will not function without a CAN interface
active. This script sets up a virtual CAN interface, allowing these scripts to
use the virtual interface and run while not on Perseus.

Any node using the CAN interface should have a parameter for changing the name
of the interface. When using a virtual interface, the name can be passed to
these nodes and they should run as if it was a real CAN interface.

## Arguments

The script takes one argument: the name of the virtual CAN interface to create.
It defaults to "vcan0" if not supplied.

## How it works

The script turns on the kernel modules
[can, vcan, and can-raw](https://www.kernel.org/doc/html/latest/networking/can.html).
Then creates a device named after the first argument (or defaulting to vcan0)
and turns it on.
