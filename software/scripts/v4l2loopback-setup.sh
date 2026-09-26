#!/bin/sh
set -euo pipefail

# This script assumes the default package manager on each distro
if lsmod | grep -q '^v4l2loopback'; then
  echo "v4l2loopback already loaded."
  exit 0
fi

. /etc/os-release

case "$ID" in
  ubuntu | debian)
    echo "Detected $PRETTY_NAME — installing v4l2loopback-dkms via apt"
    sudo apt-get update
    sudo apt-get install -y v4l2loopback-dkms
    ;;
  arch)
    echo "Detected Arch Linux — installing v4l2loopback-dkms via pacman"
    sudo pacman -S --needed --noconfirm v4l2loopback-dkms linux-headers
    ;;
  nixos)
    echo "Add this to your NixOS configuration (configuration.nix or your flake's nixosConfiguration module), then $(sudo nixos-rebuild switch) and restart the system:"
    echo "  boot.extraModulePackages = with config.boot.kernelPackages; [ v4l2loopback.out ];"
    echo '  boot.kernelModules = [ "v4l2loopback" ];'
    echo '  boot.extraModprobeConfig = "options v4l2loopback video_nr=32,33,34,35";'
    ;;
  *)
    echo "Unrecognized distro '$ID'. Install a v4l2loopback-dkms-equivalent package for your kernel, then modprobe it." >&2
    exit 1
    ;;
esac
