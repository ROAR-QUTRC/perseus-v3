# nix-setup.sh

## Motivation

This script simplifies the nix install process, adjusting the nix configuration
file to add the ROAR/ROS substituters and running the devenv-setup.sh script.

## How it works

Each part of the script checks whether it has already been run before it runs
again, ensuring there are no duplicates in the config file. It also checks if
the user is on NixOS, and prompts them to add options to their configuration.nix
if so.

In order, the script:

1.  Installing Determinate Systems Nix
    - Using the [official installer](https://determinate.systems/install/)

2.  Checking /etc/nix for nix.custom.conf (indicating Determinate Nix) and if
    nix.conf is a symlink (indicating NixOS)
3.  Checking the nix config for the ROS and ROAR substituters, appending them if
    not found.

        - [Nix configuration options](https://nix.dev/manual/nix/2.34/command-ref/conf-file.html)

4.  Same as above, but for the substituter keys.
5.  Disables the dirty git tree warning - very annoying when building with
    uncommitted changes.
6.  Runs the devenv-setup.sh script
