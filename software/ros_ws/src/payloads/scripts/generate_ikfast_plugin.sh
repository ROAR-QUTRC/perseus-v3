#!/usr/bin/env bash
#
# Regenerates the analytic IK solver in src/arm_ikfast_plugin. Run this ONLY
# when the arm's geometry changes -- the solver is baked against the link
# offsets and joint axes in arm_model.urdf and is silently wrong if they drift.
#
# Nobody needs this to build the repo. It rewrites one committed file:
#   src/arm_ikfast_plugin/src/arm_arm_ikfast_solver.cpp
# The rest of that package is ordinary hand-maintained source.
#
# Needs docker or podman (rootless is fine), or nix, which is used to get podman.
#
# translation3d: the 3-joint arm positions the wrist point (x,y,z); wrist and
# gripper are commanded directly outside IK.
#
set -euo pipefail

BASE_LINK=plate
EEF_LINK=forearm_tip
IMAGE=docker.io/personalrobotics/ros-openrave:latest
SYMPY_URL=https://files.pythonhosted.org/packages/0f/bc/104757e5baf262211bb42c10c404cb4912d206ddf596dd752ae0ae9e09ff/sympy-0.7.1.tar.gz

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$here/../../.." && pwd)
src_urdf="$repo_root/src/payloads/config/arm_model.urdf"
solver_out="$repo_root/src/arm_ikfast_plugin/src/arm_arm_ikfast_solver.cpp"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

if command -v docker >/dev/null; then
  CT=docker
elif command -v podman >/dev/null; then
  CT=podman
elif command -v nix >/dev/null; then
  CT="nix shell nixpkgs#podman --command podman"
else
  echo "Need docker, podman, or nix on PATH." >&2
  exit 1
fi
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}

# The image has no collada_urdf and Indigo's apt repos are gone, so skip COLLADA
# and write OpenRAVE's native XML straight from the URDF.
echo "==> Building OpenRAVE model"
python3 "$here/urdf_to_openrave.py" "$src_urdf" "$work/arm.xml" \
  "$BASE_LINK" "$EEF_LINK" "0 0 1"

# ikfast.py needs sympy 0.7.1; the image ships 0.7.4.1, whose matrix slicing
# breaks it. Shadow it on PYTHONPATH rather than fighting dead apt repos.
echo "==> Fetching sympy 0.7.1"
curl -sSL "$SYMPY_URL" -o "$work/sympy.tar.gz"
tar -xzf "$work/sympy.tar.gz" -C "$work"

cat >"$work/gen_ik.py" <<PY
import openravepy
from openravepy._openravepy_ import ikfast
env = openravepy.Environment()
robot = env.ReadRobotXMLFile('/work/arm.xml')
env.Add(robot)
base = [l.GetName() for l in robot.GetLinks()].index('$BASE_LINK')
eef = [l.GetName() for l in robot.GetLinks()].index('$EEF_LINK')

solver = ikfast.IKFastSolver(robot, robot)
# OpenRAVE 0.9's SolverIKChainAxisAngle.leftmultiply is unimplemented (it ends
# in assert(0)). It only runs to factor out the constant transforms ahead of
# the first joint; off, they stay in the chain and are solved inline instead.
solver.useleftmultiply = False

tree = solver.generateIkSolver(base, eef, [], solvefn=ikfast.IKFastSolver.solveFullIK_Translation3D)
open('/work/ikfast61.cpp', 'w').write(solver.writeIkSolver(tree, lang='cpp'))
PY

echo "==> Running IKFast (several minutes)"
# shellcheck disable=SC2016  # $PYTHONPATH must expand inside the container
$CT run --rm -v "$work:/work" "$IMAGE" \
  bash -c 'export PYTHONPATH=/work/sympy-0.7.1:$PYTHONPATH; cd /work && python gen_ik.py'

[ -s "$work/ikfast61.cpp" ] || {
  echo "IKFast produced no solver." >&2
  exit 1
}
cp "$work/ikfast61.cpp" "$solver_out"

echo
echo "Updated $solver_out"
echo "  colcon build --packages-select arm_ikfast_plugin"
