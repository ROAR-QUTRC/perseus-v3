#!/bin/bash
# Push this machine's clock to the rover brains over SSH.
#
# The brains have no RTC battery, so their clock is wrong on every boot, and at the
# comp there is no internet to sync from. Run this from a machine with the correct time.
#
# Usage: sync-time.sh [host...]
#
# With no arguments it reads config/network_devices.toml and targets every enabled
# "*-brain" device (by IP), skipping the machine it is run on.
#
# Needs SSH access and passwordless sudo for `date` on each host.

CONFIG="$(dirname "$(readlink -f "$0")")/../../config/network_devices.toml"

HOSTS=("$@")
if [ ${#HOSTS[@]} -eq 0 ]; then
    mapfile -t HOSTS < <(python3 -I - "${CONFIG}" "$(hostname)" <<'PY'
import sys, tomllib

with open(sys.argv[1], "rb") as f:
    devices = tomllib.load(f)
for name, dev in devices.items():
    if name.endswith("-brain") and dev.get("enabled", True) and name != sys.argv[2]:
        print(dev["ip"])
PY
    )
fi
if [ ${#HOSTS[@]} -eq 0 ]; then
    echo "No hosts to sync (none found in ${CONFIG})" >&2
    exit 1
fi

NOW="$(date -u +'%Y-%m-%d %H:%M:%S')"
echo "Setting time to ${NOW} UTC on: ${HOSTS[*]}"

LOG_DIR="$(mktemp -d)"
trap 'rm -rf "${LOG_DIR}"' EXIT

pids=()
for host in "${HOSTS[@]}"; do
    ssh -o ConnectTimeout=5 -o BatchMode=yes "${host}" "sudo -n date -u -s '${NOW}'" \
        >"${LOG_DIR}/${host}.log" 2>&1 &
    pids+=($!)
done

failed=0
for i in "${!HOSTS[@]}"; do
    if wait "${pids[$i]}"; then
        echo "  ok:     ${HOSTS[$i]} -> $(cat "${LOG_DIR}/${HOSTS[$i]}.log")"
    else
        echo "  FAILED: ${HOSTS[$i]}: $(cat "${LOG_DIR}/${HOSTS[$i]}.log")"
        failed=1
    fi
done

exit ${failed}
