# A dev shell that runs this machine's Zenoh router.
#
# Topology: every compute runs one router (rmw_zenohd). Hosts in
# software/networking/zenoh_hosts.nix belong to a site (rover or base). Routers mesh over
# unicast TCP inside their site, and only each site's bridge dials across to the other
# site's bridge. Router autoconnect is off in zenoh_router_config.json5, so gossip never
# adds a link beyond those configured here. ROS nodes run in
# peer mode against the router on localhost only (zenoh_session_config.json5). Nodes on one
# machine therefore talk directly to each other, and the routers carry everything between
# machines. Multicast scouting is off in both configs, so the host list is the whole
# topology.
#
# The static router config (zenoh_router_config.json5) is the same on every machine. The
# per-machine part, the list of routers to dial, is injected at start through
# ZENOH_CONFIG_OVERRIDE. That variable is set only inside the router process. ROS sessions
# read the same variable, so it must never be exported to the shell.
#
# Entering this profile starts the router in the background if nothing already listens on
# its port, so a second shell reuses the running router. `devenv shell` never starts
# `processes` entries (only `devenv up` does), hence the explicit start in enterShell.
{
  pkgs,
  config,
  lib,
  ...
}:
let
  # Fill in defaults and refuse a host list that breaks the topology rules.
  listedHosts = lib.mapAttrs (
    name: host:
    if !(host ? ip) || !(host ? site) then
      throw "software/networking/zenoh_hosts.nix: ${name} needs both `ip` and `site`"
    else
      {
        bridge = false;
        port = 7447;
      }
      // host
  ) (import ../../software/networking/zenoh_hosts.nix);

  sites = lib.unique (lib.mapAttrsToList (_: host: host.site) listedHosts);
  bridgesOf =
    site: lib.attrNames (lib.filterAttrs (_: host: host.site == site && host.bridge) listedHosts);
  crowdedSites = lib.filter (site: lib.length (bridgesOf site) > 1) sites;

  hosts =
    if crowdedSites != [ ] then
      throw (
        "software/networking/zenoh_hosts.nix: a site may have only one bridge, but "
        + lib.concatMapStringsSep "; " (
          site: "${site} has ${lib.concatStringsSep ", " (bridgesOf site)}"
        ) crowdedSites
      )
    else
      listedHosts;

  endpointOf = host: "tcp/${host.ip}:${toString host.port}";

  # Topology rule: a router dials every other host in its own site, and a bridge also dials
  # the bridges of the other sites. Nothing else crosses between sites.
  dials = me: host: host.site == me.site || (me.bridge && host.bridge);

  # JSON list of the routers that host `self` (described by `me`) dials.
  peersOf =
    self: me:
    builtins.toJSON (
      lib.mapAttrsToList (_: endpointOf) (lib.filterAttrs (n: host: n != self && dials me host) hosts)
    );

  siteHasBridge = site: lib.boolToString (bridgesOf site != [ ]);

  # Sets zenoh_peers, zenoh_ip, zenoh_port, zenoh_site, zenoh_bridge and
  # zenoh_site_has_bridge for the host named by $zenoh_host. A host that is not listed joins
  # site ${ZENOH_SITE:-base} as a non-bridge and listens on the default port.
  resolveHost = ''
    zenoh_ip=""
    zenoh_port=7447
    zenoh_bridge=false
    case "$zenoh_host" in
    ${lib.concatStrings (
      lib.mapAttrsToList (name: host: ''
        ${lib.escapeShellArg name})
          zenoh_peers=${lib.escapeShellArg (peersOf name host)}
          zenoh_ip=${host.ip}
          zenoh_port=${toString host.port}
          zenoh_site=${lib.escapeShellArg host.site}
          zenoh_bridge=${lib.boolToString host.bridge}
          zenoh_site_has_bridge=${siteHasBridge host.site}
          ;;
      '') hosts
    )}
      *)
        zenoh_site=''${ZENOH_SITE:-base}
        case "$zenoh_site" in
        ${lib.concatMapStrings (site: ''
          ${lib.escapeShellArg site})
            zenoh_peers=${
              lib.escapeShellArg (
                peersOf "" {
                  inherit site;
                  bridge = false;
                }
              )
            }
            zenoh_site_has_bridge=${siteHasBridge site}
            ;;
        '') sites}
          *)
            zenoh_peers='[]'
            zenoh_site_has_bridge=false
            ;;
        esac
        ;;
    esac
  '';

  state = config.env.DEVENV_STATE;
  logFile = "${state}/zenoh-router.log";
  pidFile = "${state}/zenoh-router.pid";
in
{
  packages = [
    pkgs.iproute2
    pkgs.util-linux
  ];

  scripts.zenoh-router = {
    description = "Run this machine's Zenoh router in the foreground, dialling the hosts in zenoh_hosts.nix";
    exec = ''
      # ZENOH_HOST pretends to be another host from the list, and ZENOH_SITE picks the site
      # of an unlisted host (both for testing on one machine).
      zenoh_host=''${ZENOH_HOST:-$(hostname)}
      ${resolveHost}

      # ZENOH_ROUTER_EXTRA_OVERRIDE appends further "key=value" pairs, separated by ';'.
      export ZENOH_CONFIG_OVERRIDE="connect/endpoints=$zenoh_peers''${ZENOH_ROUTER_EXTRA_OVERRIDE:+;$ZENOH_ROUTER_EXTRA_OVERRIDE}"
      export RUST_LOG=''${ZENOH_ROUTER_LOG:-zenoh=info}

      echo "zenoh-router: host $zenoh_host, site $zenoh_site (bridge: $zenoh_bridge), dialling $zenoh_peers"
      # Run the binary directly rather than through `ros2 run`, so the PID in the pid file
      # is the router itself and a signal reaches it.
      exec "$(ros2 pkg prefix rmw_zenoh_cpp)/lib/rmw_zenoh_cpp/rmw_zenohd" "$@"
    '';
  };

  scripts.zenoh-router-stop = {
    description = "Stop the background Zenoh router started by this profile";
    exec = ''
      if [ -f ${pidFile} ] && pid=$(cat ${pidFile}) && kill -0 "$pid" 2>/dev/null; then
        kill "$pid" && rm -f ${pidFile} && echo "zenoh-router: stopped (pid $pid)"
      else
        rm -f ${pidFile}
        echo "zenoh-router: no background router from this profile is running"
      fi
    '';
  };

  enterShell = ''
    echo -e "\e[38;5;208mZenoh: router per machine, sites and peers from software/networking/zenoh_hosts.nix\e[0m"

    zenoh_host=$(hostname)
    ${resolveHost}

    if [ -z "$zenoh_ip" ]; then
      echo -e "\e[33m  WARNING: $zenoh_host is not in software/networking/zenoh_hosts.nix.\e[0m"
      echo    "           Its router joins site $zenoh_site (set ZENOH_SITE to change), but no other router dials it."
    elif ! ip -brief addr show | grep -qw "$zenoh_ip"; then
      zenoh_avail=$(ip -brief -4 addr show | awk '$2 == "UP" { printf "%s ", $3 }')
      echo -e "\e[33m  WARNING: $zenoh_host is listed as $zenoh_ip, but no interface here holds it.\e[0m"
      echo    "           Other routers cannot reach this one. Addresses up here: $zenoh_avail"
    fi

    if ss -Hltn "sport = :$zenoh_port" | grep -q .; then
      echo    "  router:  already listening on :$zenoh_port"
    else
      mkdir -p ${lib.escapeShellArg state}
      setsid zenoh-router >${logFile} 2>&1 </dev/null &
      echo $! >${pidFile}
      disown
      echo    "  router:  started on :$zenoh_port (pid $(cat ${pidFile}), stop with zenoh-router-stop)"
    fi
    if [ "$zenoh_site_has_bridge" != true ]; then
      echo -e "\e[33m  WARNING: site $zenoh_site has no bridge, so it cannot reach the other sites.\e[0m"
    fi
    if [ "$zenoh_bridge" = true ]; then
      echo  "  site:    $zenoh_site (bridge)"
    else
      echo  "  site:    $zenoh_site"
    fi
    echo    "  peers:   $zenoh_peers"
    echo    "  log:     ${logFile}"

    unset zenoh_host zenoh_ip zenoh_port zenoh_peers zenoh_avail zenoh_site zenoh_bridge zenoh_site_has_bridge
  '';
}
