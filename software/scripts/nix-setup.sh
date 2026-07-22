#!/bin/sh

if [ "$(id -u)" -eq 0 ]; then
  echo "Please run as yourself! Running as root breaks the setup."
  exit 1
fi

if ! command -v nix >/dev/null 2>&1; then
  echo "Setting up Nix"
  # install nix! https://github.com/DeterminateSystems/nix-installer
  # using the Determinate Systems installer rather than the "official" installer for its uninstall options and better error handling
  curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
else
  echo "Nix already present on system! Continuing"
fi

NIX_CONFIG_FILE_PATH="/etc/nix/nix.conf"
DET_NIX_CONFIG_FILE_PATH="/etc/nix/nix.custom.conf"
if test -f "$DET_NIX_CONFIG_FILE_PATH"; then
  NIX_CONFIG_FILE_PATH="$DET_NIX_CONFIG_FILE_PATH"
fi

if [ -L "$NIX_CONFIG_FILE_PATH" ]; then
  echo "Nix config file is a symlink. Assuming NixOS"
  NIXOS=1
else
  echo "Nix config file $NIX_CONFIG_FILE_PATH is not a symlink. Continuing"
fi

ROS_SUB="https://ros.cachix.org"
ROAR_SUB="https://roar-qutrc.cachix.org"

ROS_TRUSTED_SUBSTITUTER_PATTERN=".*trusted-substituters = .*?$ROS_SUB"
ROAR_TRUSTED_SUBSTITUTER_PATTERN=".*trusted-substituters = .*?$ROAR_SUB"

# Check if nix config includes both the ros and roar subsituters
if grep -Eq "$ROS_TRUSTED_SUBSTITUTER_PATTERN" "$NIX_CONFIG_FILE_PATH" && grep -Eq "$ROAR_TRUSTED_SUBSTITUTER_PATTERN" "$NIX_CONFIG_FILE_PATH"; then
  echo "Extra trusted substituters already present!"
else
  if [ -z "$NIXOS" ]; then
    # TODO: Add logic for adding just one substituter if the other is already present
    echo "Adding trusted substituters to nix configuration"
    EXTRA_TRUSTED_SUBSTITUTERS="extra-trusted-substituters = $ROAR_SUB $ROS_SUB"
    echo "$EXTRA_TRUSTED_SUBSTITUTERS" | sudo tee -a "$NIX_CONFIG_FILE_PATH"
    RESTART_NIX_DAEMON=true
  else
    echo "Add the following to your configuration.nix:"
    echo "nix.settings.trusted-substituters = [ $ROAR_SUB $ROS_SUB ];"
  fi
fi

# Note the escaping of the '+' character
ROS_SUB_KEY="ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL\+38Rvo="
ROAR_SUB_KEY="roar-qutrc.cachix.org-1:ZKgHZSSHH2hOAN7\+83gv1gkraXze5LSEzdocPAEBNnA="

ROS_TRUSTED_KEY_PATTERN=".*?trusted-public-keys = .*?$ROS_SUB_KEY"
ROAR_TRUSTED_KEY_PATTERN=".*?trusted-public-keys = .*?$ROAR_SUB_KEY"
if grep -Eq "$ROAR_TRUSTED_KEY_PATTERN" "$NIX_CONFIG_FILE_PATH" && grep -Eq "$ROS_TRUSTED_KEY_PATTERN" "$NIX_CONFIG_FILE_PATH"; then
  echo "Extra trusted keys already present!"
else
  if [ -z "$NIXOS" ]; then
    echo "Adding extra trusted keys..."
    EXTRA_TRUSTED_KEYS="extra-trusted-public-keys = $ROAR_SUB_KEY $ROS_SUB_KEY"
    echo "$EXTRA_TRUSTED_KEYS" | sudo tee -a "$NIX_CONFIG_FILE_PATH"
    RESTART_NIX_DAEMON=true
  else
    echo "Add the following to your configuration.nix:"
    echo "nix.settings.trusted-public-keys = [ $ROAR_SUB_KEY $ROS_SUB_KEY ];"
  fi
fi

DISABLE_WARN_DIRTY="warn-dirty = false"
if grep -Fq "$DISABLE_WARN_DIRTY" "$NIX_CONFIG_FILE_PATH"; then
  echo "Dirty git tree warning already disabled!"
else
  if [ -z "$NIXOS" ]; then
    echo "Disabling dirty git tree warning"
    echo "$DISABLE_WARN_DIRTY" | sudo tee -a "$NIX_CONFIG_FILE_PATH"
    RESTART_NIX_DAEMON=true
  else
    echo "Add the following to your configuration.nix:"
    echo "nix.settings.$DISABLE_WARN_DIRTY"
  fi
fi

if [ ! -z "$RESTART_NIX_DAEMON" ]; then
  echo "Restarting nix daemon"
  sudo systemctl restart nix-daemon
fi

"$(git rev-parse --show-toplevel)/software/scripts/devenv-install.sh" || exit
. "$(git rev-parse --show-toplevel)/software/scripts/devenv-setup.sh"
