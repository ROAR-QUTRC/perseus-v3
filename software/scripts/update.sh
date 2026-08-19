#!/bin/sh

cd "$(git rev-parse --show-toplevel)/nix/extra-packages/ros-packages/third-party/" || exit 1

ros-NUR-helper.py -c ./generator.toml
