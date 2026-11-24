# Shared Libraries

The directory `software/shared/` contains several libraries and utility classes. This section explains what each library does and when you should use it.

!!! tip

    Shared libraries can be made available to your ROS2 package and nodes by including them as a dependency in your package's package.xml and then running the script 'nix-package.sh'

## Hi-CAN

Abbreviated from "hierarchical CAN", Hi-CAN is the library implementing the standards laid out [here](./hi-can.md), and is shared across ROS and native code, as well as firmware.
The main library contains the code defining the main interfaces with which code will interact with the library, as well as all of the devices on the bus and their parameters.

!!! warning

    Since this particular library is shared between both the ROS code _and_ the firmware, it needs to be written in pure C++ (no external dependencies).

However, this on its own is not particularly useful - which is where implementations come in.
The `hi-can-raw` library implements `hi_can::FilteredCanInterface` using the Linux SocketCAN [`RAW_CAN`](https://docs.kernel.org/networking/can.html#raw-protocol-sockets-with-can-filters-sock-raw) interface, and is what most code uses to interface with the CAN bus.
The `hi-can-net` library is currently unused, but may become an implementation of `hi_can::CanInterface` which forwards all traffic over a network connection.

## Simple-networking

The simple-networking library provides a modern C++ implementation for handling network socket communications, with a primary focus on client-side operations.

It offers an object-oriented wrapper around traditional POSIX socket operations, supporting both TCP and UDP protocols. The library implements RAII principles through its `networking::Client` class, which manages socket creation, configuration, connection and cleanup while providing exception-based error handling for robust failure management.

The library distinguishes itself through flexible socket configuration using handler callbacks, support for custom bind addresses and a clean abstraction over low-level socket operations. It provides convenient methods for transmitting and receiving both string and binary data, with support for both blocking and non-blocking operations. Error handling is comprehensive, with descriptive error messages that include both the operation context and underlying system error details.

!!! warning

    This library is not used for ROS2 communications, it exists for scenarios such as a creating a ROS2 driver node which needs to interface with a specific sensor via ethernet.

## Utility Classes

### File Descriptor Wrapper

When opening files, sockets, pipes or devices standard on unix systems an integer is returned known as a File Descriptor (FD). In short, when a process opens or uses one of the affore mentioned resources, the operating system must keep track of this and the FD is simply a unique identifier for a resource. For more information and graphical explanations check out this [article](https://bottomupcs.com/ch01s03.html).

The `FdWrapper` class located in `software/shared/fd-wrapper/` is a wrapper around a FD which implements the [RAII](../03-standards/01-software-standards/02-c-standards.md#always-use-raii-or-riia-and-understand-who-owns-what) design pattern and handles automatically closing resources.

!!! tip "Usage Example"

    A good example of how this is used can be found in the `RawCanInterface` class.

### Pointer Wrapper

The `PtrWrapper` class is a wrapper around a pointer which will automatically deallocate the resource when it goes out of scope. Intended for use in situations where [shared_ptr](https://en.cppreference.com/cpp/memory/shared_ptr) can't be used, such as when the resource is not allocated with `new`.
