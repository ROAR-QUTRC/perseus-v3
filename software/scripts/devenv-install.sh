#!/bin/sh

if command -v devenv >/dev/null 2>&1; then
  echo "Devenv already installed!"
  exit 0
fi

if ! command -v nix >/dev/null 2>&1; then
  echo "Nix is not installed! Run the nix-setup script"
  exit 1
fi

if command -v home-manager >/dev/null 2>&1; then
  echo "User is using Home Manager! Add the following to your home.nix:"
  echo "home.packages = with pkgs; [ devenv ];"
  exit 1
fi

if command -v nixos-rebuild >/dev/null 2>&1; then
  echo "User is on NixOS! Add the following to your configuration.nix:"
  echo "environment.systemPackages = with pkgs; [ devenv ];"
  exit 1
fi

echo "Installing devenv"
nix-env -iA devenv -f https://github.com/NixOS/nixpkgs/tarball/nixpkgs-unstable
