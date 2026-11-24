# Zenoh networking

ROS 2 on Perseus runs on `rmw_zenoh_cpp`. Every compute runs its own Zenoh router, and ROS
nodes only ever talk to the router on their own machine. The routers carry everything
between machines.

Machines belong to one of two **sites**: the rover and the base station. Inside a site,
every router connects to every other router. Between sites there is exactly one link, from
one **bridge** to the other bridge.

```
        rover site                              base station site

 medium-brain    small-brain               BoMachine
  [router] <-----> [router]                 [router]
      \             /                           |
       \           /                            |
       [  router  ] <====== radio link ======> [ router ]
        big-brain                                cookie
        (bridge)                                (bridge)
```

A node on medium-brain reaches a node on BoMachine through big-brain and cookie. The routers
forward the traffic. No other machine sends anything across the radio link.

## Quick start

On each machine, open the `zenoh` profile:

```sh
devenv --profile zenoh shell
```

Entering the shell starts this machine's router in the background. Then run ROS as normal,
in this shell or any other `zenoh` shell on the same machine:

```sh
ros2 launch ...
```

The banner tells you what the router is doing:

```
Zenoh: router per machine, sites and peers from software/networking/zenoh_hosts.nix
  router:  started on :7447 (pid 12345, stop with zenoh-router-stop)
  site:    rover (bridge)
  peers:   ["tcp/192.168.2.50:7447","tcp/192.168.2.12:7447","tcp/192.168.2.15:7447"]
  log:     .devenv/profiles/zenoh/state/zenoh-router.log
```

- `router:` either `started` (this shell launched it) or `already listening` (another shell
  or process already runs a router on that port, and this shell reuses it).
- `site:` the site this machine belongs to, and whether it is the bridge.
- `peers:` the routers this machine dials.
- `log:` where the background router writes its output.

Yellow `WARNING` lines mean something in the setup is wrong. See
[Troubleshooting](#troubleshooting).

### Start order

Machines can start in any order. A router keeps retrying its peers until they come up, and
reconnects when a peer restarts.

Start the router before any ROS node on that machine. The `zenoh` shell does this for you.
Other profiles (`dev`, `autonomy`, ...) do **not** start a router. If you work in one of
those, start the router first from a `zenoh` shell, or the nodes have no router to connect
to.

## Commands

| Command             | What it does                                                                          |
| ------------------- | ------------------------------------------------------------------------------------- |
| `zenoh-router`      | Run this machine's router in the foreground (for example in a zellij pane). Ctrl-C stops it. |
| `zenoh-router-stop` | Stop the background router that a `zenoh` shell started.                              |

Use one or the other. If a background router is already running, `zenoh-router` in the
foreground fails because the port is in use.

## Check that it works

On one machine, start a talker:

```sh
ros2 run demo_nodes_cpp talker
```

On another machine, in the same site or across the bridge, start a listener:

```sh
ros2 run demo_nodes_cpp listener
```

The listener should print `I heard: [Hello World: N]`.

To see which routers this machine is connected to:

```sh
ss -Htn state established '( sport = :7447 or dport = :7447 )'
```

Expect one connection to each peer in the banner, plus one per local ROS node. On a bridge,
one of them is the far site's bridge.

To follow the router log:

```sh
tail -f .devenv/profiles/zenoh/state/zenoh-router.log
```

## The host file

`zenoh_hosts.nix` is the only file to edit when machines change. Each entry is keyed by the
machine's hostname (exactly what `hostname` prints):

```nix
"big-brain" = {
  ip = "192.168.2.11";
  site = "rover";
  bridge = true;
};
"medium-brain" = {
  ip = "192.168.2.12";
  site = "rover";
};
```

| Field    | Required | Meaning                                                                   |
| -------- | -------- | ------------------------------------------------------------------------- |
| `ip`     | yes      | Address the other routers dial to reach this machine.                     |
| `site`   | yes      | `"rover"` or `"base"`. Machines with the same site connect to each other. |
| `bridge` | no       | `true` on the one machine per site that links to the other site.          |
| `port`   | no       | Router port, default `7447`.                                              |

The rules the file follows:

- A router dials every other machine in its own site.
- A bridge also dials the bridge of every other site.
- Each site has at most one bridge. The shell refuses to start if a site has two:
  `error: ... a site may have only one bridge, but rover has big-brain, medium-brain`.
- A site with no bridge works on its own but cannot reach the other site.

### Add a machine

1. Add an entry to `zenoh_hosts.nix` with its hostname, `ip` and `site`.
2. On **every** machine, apply the new list:

   ```sh
   zenoh-router-stop
   exit
   devenv --profile zenoh shell
   ```

   A running router keeps the peer list it started with, and a new shell reuses a router
   that is already running. A restart is the only way to pick up the change.

### Change which machine is the bridge

Move `bridge = true` to the new machine in the same site, then restart the routers as above.
Restart both bridges, old and new, and the far site's bridge, because their peer lists all
change.

### A machine that is not in the file

A laptop that is not listed still works. It joins the site in `$ZENOH_SITE` (default
`base`) as a non-bridge, and dials that site's machines. Nothing dials it back, and the
banner warns about it. To join the rover site instead:

```sh
ZENOH_SITE=rover devenv --profile zenoh shell
```

Add the machine to the file if it is going to stay.

## Troubleshooting

**`WARNING: <host> is not in software/networking/zenoh_hosts.nix`**
The hostname is not listed. The machine still joins `$ZENOH_SITE`, but other routers do not
dial it. Add it to the file if it should be permanent.

**`WARNING: <host> is listed as <ip>, but no interface here holds it`**
The machine is not on the address the file gives. Other routers dial that address and fail.
This is expected when you are off the rover network. On the network, fix the IP in the file
or the machine's network config.

**`WARNING: site <site> has no bridge`**
No machine in this site has `bridge = true`, so it cannot reach the other site.

**`router: already listening`, but nothing reaches other machines**
Something else holds port 7447, often an `rmw_zenohd` started by hand without the peer
list. The shell reuses it instead of starting the right one. Find it and stop it:

```sh
ss -Hltnp 'sport = :7447'
kill <pid>
```

Then re-enter the `zenoh` shell.

**Nodes on one machine work, but nothing crosses to the other site**
Check that both bridges are running. On each bridge, the `ss` command above should show a
connection to the other bridge's IP. If it does not, check that the radio link is up
(`ping` the other bridge) and read the router log.

**A ROS node hangs at startup**
It is waiting for a router on `localhost:7447`. Start one with a `zenoh` shell or
`zenoh-router`.

**More log output**
Set the level before starting the router or the node:

```sh
ZENOH_ROUTER_LOG=zenoh=debug zenoh-router   # router
RUST_LOG=zenoh=debug ros2 run ...           # a ROS node
```

## Test on one machine

You can stand in for another host, and move the router to spare ports, so a test does not
touch a real router:

```sh
# router A dials B; router B listens only
ZENOH_HOST=testA ZENOH_ROUTER_EXTRA_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7450"];connect/endpoints=["tcp/127.0.0.1:7451"]' zenoh-router &
ZENOH_HOST=testB ZENOH_ROUTER_EXTRA_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7451"];connect/endpoints=[]' zenoh-router &

# talker on A, listener on B
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7450"]' ros2 run demo_nodes_cpp talker &
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7451"]' ros2 run demo_nodes_cpp listener
```

Set `ZENOH_HOST=big-brain` (or any listed name) to see the exact peer list that machine
would use. The first line the router prints shows it.

## Reference

### Files

| File                           | Purpose                                                               |
| ------------------------------ | --------------------------------------------------------------------- |
| `zenoh_hosts.nix`              | Hostname, IP, site and bridge flag for every router. Edit this one.   |
| `zenoh_router_config.json5`    | Static router config, the same on every machine. Leave `connect.endpoints` empty. |
| `zenoh_session_config.json5`   | Config for every ROS node: peer mode, connects to `localhost:7447`.   |
| `../../nix/profiles/zenoh.nix` | The `zenoh` devenv profile: turns the host file into peer lists and starts the router. |
| `../../nix/networking.nix`     | Sets `RMW_IMPLEMENTATION` and the two config paths for every profile. |

### Environment variables

| Variable                      | Effect                                                                                                    |
| ----------------------------- | --------------------------------------------------------------------------------------------------------- |
| `ZENOH_ROUTER_LOG`            | Router log level, default `zenoh=info`.                                                                   |
| `RUST_LOG`                    | Log level for ROS nodes, default `zenoh=warn`.                                                            |
| `ZENOH_HOST`                  | Make `zenoh-router` act as another listed host (testing).                                                 |
| `ZENOH_SITE`                  | Site that an unlisted machine joins, default `base`.                                                      |
| `ZENOH_ROUTER_EXTRA_OVERRIDE` | Extra `key=value` pairs for the router, separated by `;`, applied after the peer list.                    |

The router gets its peer list through `ZENOH_CONFIG_OVERRIDE`. Never export that variable
in a shell that runs ROS nodes, because the nodes read it too.

### How it works

- Multicast discovery is off in both configs, so the host file is the whole topology.
- Router autoconnect is off in `zenoh_router_config.json5`. Routers tell each other about
  other routers, but never dial one that their own peer list does not name. That is what
  keeps the radio link down to one connection.
- `devenv shell` never starts devenv `processes` (only `devenv up` does). So the profile
  starts the router itself, in the background with `setsid`, and the router outlives the
  shell. Stop it with `zenoh-router-stop`.

### Limitation and next step

This controls which routers dial each other, but it does not block anything. Non-bridge
routers still accept connections on `[::]:7447`, so a wrongly configured machine can still
dial in across the link.

Zenoh 1.9 added regions (`region_name`, `gateway.south`), which would make the site
boundary part of Zenoh itself. The pinned rmw_zenoh (0.2.10) bundles zenoh 1.8.0, which has
no regions. rmw_zenoh 0.2.12 bundles zenoh 1.10.1. After that upgrade, `site` maps to
`region_name` and each bridge becomes a region gateway.
