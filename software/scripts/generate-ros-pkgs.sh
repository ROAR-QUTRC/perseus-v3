#!/bin/sh

TOP_LEVEL="$(git rev-parse --show-toplevel || exit)"

# cd to the nix directory
if ! cd "$TOP_LEVEL/nix"; then
  echo "Error when entering nix directory"
  exit
fi

ROS_WS="$TOP_LEVEL/software/ros_ws"
OUTPUT_DIR="$TOP_LEVEL/nix/extra-packages/ros-packages/perseus"

echo "Cleaning old packaging files"
rm -r "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

echo "Running ros2nix to generate new packaging files..."
# need to keep the environment isolated so that direnv doesn't screw with ros2nix.
# -i (--ignore-environment) clears all environment variables and drops you into a shell
# --command automatically runs the following command and exits the shell once it completes
# nix run is the "better" way to do this, but it doesn't allow for complete environment isolation like the shell -i flag.
# we want word splitting here to get all the package files in one go, so disable the shellcheck warning
# shellcheck disable=SC2046
if ! nix shell -i github:wentasah/ros2nix --command ros2nix --output-dir="$OUTPUT_DIR" --output-as-nix-pkg-name --no-default --distro jazzy $(find "$ROS_WS" -type d \( -name log -o -name build -o -name install \) -prune -o -name 'package.xml' -print); then
  echo "Error when generating nix derivations"
  exit
fi

echo "Formatting generated files"
if ! cd "$OUTPUT_DIR"; then
  echo "Error when entering generated packages output directory"
  exit
fi
treefmt "$OUTPUT_DIR"

echo "$*"
if [ "--no-commit" = "$1" ]; then
  echo "Will not commit changes"
  exit 0
fi

cleanup() {
  if [ -n "$HAS_GIT_STAGING" ]; then
    echo "Restoring git stash"
    git stash pop >/dev/null
  fi
}

trap cleanup EXIT

# Pathspec: https://git-scm.com/docs/gitglossary#Documentation/gitglossary.txt-aiddefpathspecapathspec
if ! git diff --cached --quiet -- "$TOP_LEVEL" ":!${OUTPUT_DIR}" >/dev/null; then
  HAS_GIT_STAGING=1
  echo "Stashing staged changes"
  git stash push -S -- "$TOP_LEVEL" ":!$OUTPUT_DIR" >/dev/null
fi

echo "Staging changes"
git add .
echo "Committing changes"
git commit -m "Chore: Update Nix packaging" >/dev/null
