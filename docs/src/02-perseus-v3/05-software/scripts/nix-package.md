# nix-package.sh

## Motivation

This script autogenerates nix derivations for the ROS2 packages in
`software/ros_ws`. It reads the `package.xml` files and uses
[ROS2nix](https://github.com/wentasah/ros2nix) to create the derivations.

This is much easier and faster than creating the nix derivations manually, and
it is less prone to errors. It also separates the changes neatly into one
commit.

## Arguments

If '--no-commit' is passed, the script will exit after formatting the files
without committing the changed derivations.

## How it works

1. Removes all of the old derivations from `nix/ros-packages/perseus/`
2. Runs ROS2nix on all `package.xml` files in the `software/ros_ws` directory.
3. Runs treefmt on the generated files to ensure they are formatted with nixfmt.
4. If '--no-commit' is not passed, it continues and commits the changes.
5. Using
   [pathspec](https://git-scm.com/docs/gitglossary#Documentation/gitglossary.txt-aiddefpathspecapathspec),
   stashes any staged changes other than the generated packages
6. Commits the generated packages using the commit message "Chore: Update Nix
   packaging"
