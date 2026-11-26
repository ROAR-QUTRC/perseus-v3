# Groot2: BehaviorTree.CPP editor and live monitor (https://www.behaviortree.dev/groot)
# Proprietary freeware, only distributed as an AppImage, so it is wrapped rather than built.
{
  lib,
  stdenv,
  appimageTools,
  fetchurl,
}:
let
  pname = "groot2";
  version = "1.9.0";
  sources = {
    x86_64-linux = {
      arch = "x86_64";
      hash = "sha256-JtBAJVyxRixlEZVPNF7nMoDXLH+IovnRO7r/mQLebHA=";
    };
    aarch64-linux = {
      arch = "aarch64";
      hash = "sha256-YM7q/sWRVXMmpK/sKIFrQK51th21cSaxOSrnK3i6ZhU=";
    };
  };
  source =
    sources.${stdenv.hostPlatform.system}
      or (throw "groot2: unsupported system ${stdenv.hostPlatform.system}");
  src = fetchurl {
    url = "https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v${version}-${source.arch}.AppImage";
    inherit (source) hash;
  };
  appimageContents = appimageTools.extractType2 { inherit pname version src; };
in
appimageTools.wrapType2 {
  inherit pname version src;

  # The sandbox passes the host's /etc/fonts through but replaces /usr, so on a non-NixOS
  # host /usr/share/fonts (which that config names) is empty and every glyph renders as a
  # box. Shipping a font in the sandbox's own /usr/share/fonts gives the host config one.
  extraPkgs = pkgs: [ pkgs.dejavu_fonts ];

  # Every launch looks like a fresh install on a fresh machine. Groot2 embeds the Cryptlex
  # LexActivator DRM, which fingerprints the host (machine-id, DMI serials, MAC addresses,
  # hostname), leaves obfuscated files in ~/.local/share/{lexdev,data/bconf} and
  # /var/tmp/lexdev, and the app sends telemetry. The stock wrapper binds almost the whole
  # host read-write, so instead:
  #  - every host top-level dir but /run and /sys becomes an empty tmpfs (home, /var, /tmp,
  #    /root, /opt, ...) and only the git checkout groot2 was started from is bound back, so
  #    nothing outside that checkout persists between sessions
  #  - machine-id is random per launch, DMI and /sys/class/net are hidden, hostname is
  #    localhost, and the host environment (tokens, agent sockets) is dropped
  #  - the DRM and telemetry hosts resolve to nothing; other networking still works so the
  #    live monitor can reach a running tree
  privateTmp = true;
  unshareUts = true;
  chdirToPwd = false;
  extraPreBwrapCmds = ''
    groot2_args=(--clearenv)
    for var in DISPLAY LANG LANGUAGE TZ TERM XCURSOR_SIZE XCURSOR_THEME QT_SCALE_FACTOR \
      QT_SCREEN_SCALE_FACTORS QT_AUTO_SCREEN_SCALE_FACTOR QT_ENABLE_HIGHDPI_SCALING QT_FONT_DPI \
      "''${!LC_@}"; do
      [[ -n ''${!var:-} ]] && groot2_args+=(--setenv "$var" "''${!var}")
    done
    groot2_user=''${USER:-$(id -un)}
    # the bundled Qt has no wayland plugin and the wayland socket is hidden below
    groot2_args+=(--setenv HOME "$HOME" --setenv USER "$groot2_user" --setenv LOGNAME "$groot2_user"
      --setenv PATH /usr/bin:/usr/sbin --setenv QT_QPA_PLATFORM xcb)

    for dir in /*; do
      case $dir in
        /nix | /dev | /proc | /etc | /run | /sys | /tmp | /usr | /bin | /sbin | /lib | /lib32 | /lib64 | /libexec) ;;
        *) [[ -d $dir ]] && groot2_args+=(--tmpfs "$dir") ;;
      esac
    done
    groot2_args+=(--dir "$HOME" --dir /var/tmp)
    for dir in /run/user /run/media /sys/devices/virtual/dmi /sys/firmware/dmi /sys/class/net; do
      [[ -d $dir ]] && groot2_args+=(--tmpfs "$dir")
    done
    if [[ ''${XDG_RUNTIME_DIR:-} == /run/user/* ]]; then
      groot2_args+=(--perms 0700 --dir "$XDG_RUNTIME_DIR" --setenv XDG_RUNTIME_DIR "$XDG_RUNTIME_DIR")
    fi
    if [[ -n ''${XAUTHORITY:-} && -e $XAUTHORITY ]]; then
      groot2_args+=(--ro-bind "$XAUTHORITY" "$XAUTHORITY" --setenv XAUTHORITY "$XAUTHORITY")
    fi

    # stop the wrapper symlinking these to the host's copies; bwrap can't mount over its
    # absolute symlinks, so if this stops applying the launch fails rather than leaking
    etc_ignored+=(/etc/machine-id /etc/hosts /etc/nsswitch.conf)
    exec {groot2_machine_id}< <(printf '%08x%08x%08x%08x\n' $SRANDOM $SRANDOM $SRANDOM $SRANDOM)
    exec {groot2_hosts}< <(
      cat /etc/hosts 2>/dev/null
      for host in api.cryptlex.com api.eu.cryptlex.com api.dev.cryptlex.com \
        fjzaugiiitqpekwgheev.supabase.co ip-api.com; do
        printf '0.0.0.0 %s\n:: %s\n' "$host" "$host"
      done
    )
    # host nsswitch with /etc/hosts consulted first, so nss-resolve can't skip the blocks
    exec {groot2_nsswitch}< <(
      if [[ -r /etc/nsswitch.conf ]]; then
        while IFS= read -r line; do
          if [[ $line == hosts:* ]]; then
            line=" ''${line#hosts:} "
            line="hosts: files''${line// files / }"
          fi
          printf '%s\n' "$line"
        done </etc/nsswitch.conf
      else
        echo 'hosts: files dns'
      fi
    )
    groot2_args+=(--ro-bind-data "$groot2_machine_id" /etc/machine-id
      --ro-bind-data "$groot2_hosts" /etc/hosts
      --ro-bind-data "$groot2_nsswitch" /etc/nsswitch.conf --hostname localhost)

    # share the enclosing git checkout (the behaviour trees), but never $HOME or a parent of it
    groot2_share=$PWD
    while [[ $groot2_share != / && ! -e $groot2_share/.git ]]; do
      groot2_share=''${groot2_share%/*}
      groot2_share=''${groot2_share:-/}
    done
    [[ $groot2_share == / ]] && groot2_share=$PWD
    if [[ $groot2_share == / || $HOME/ == "$groot2_share"/* ]]; then
      groot2_args+=(--chdir "$HOME")
    else
      groot2_args+=(--bind "$groot2_share" "$groot2_share" --chdir "$PWD")
    fi
  '';
  extraBwrapArgs = [ "\"\${groot2_args[@]}\"" ];

  extraInstallCommands = ''
    for desktop in ${appimageContents}/*.desktop; do
      install -Dm444 "$desktop" $out/share/applications/${pname}.desktop
    done
    substituteInPlace $out/share/applications/${pname}.desktop \
      --replace-quiet 'Exec=AppRun' 'Exec=${pname}'
    if [ -d ${appimageContents}/usr/share/icons ]; then
      cp -r ${appimageContents}/usr/share/icons $out/share/
    fi
  '';

  meta = {
    description = "Editor and live monitor for BehaviorTree.CPP behavior trees";
    homepage = "https://www.behaviortree.dev/groot";
    license = lib.licenses.unfree;
    sourceProvenance = [ lib.sourceTypes.binaryNativeCode ];
    platforms = builtins.attrNames sources;
    mainProgram = pname;
  };
}
