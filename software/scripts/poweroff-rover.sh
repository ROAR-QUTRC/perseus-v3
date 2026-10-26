#!/bin/bash
# Power off the rover brains over SSH, so compute can be switched off at the RCB.
#
# Wait for every host to report "down" before holding the compute power button.
#
# Usage: poweroff-rover.sh <ip> [ip...]
#
# Needs SSH access and passwordless sudo for `poweroff` on each host, e.g. in
# /etc/sudoers.d/poweroff:  qutrc ALL=(root) NOPASSWD: /usr/sbin/poweroff
# Connects as $SSH_USER (default: qutrc).

SSH_USER="${SSH_USER:-qutrc}"
DOWN_TIMEOUT=60

HOSTS=("$@")
if [ ${#HOSTS[@]} -eq 0 ]; then
  echo "Usage: $(basename "$0") <ip> [ip...]" >&2
  exit 1
fi

echo "Powering off: ${HOSTS[*]}"

LOG_DIR="$(mktemp -d)"
trap 'rm -rf "${LOG_DIR}"' EXIT

# Check sudo up front, then detach the poweroff so ssh returns before the link dies.
pids=()
for host in "${HOSTS[@]}"; do
  ssh -o ConnectTimeout=5 -o BatchMode=yes "${SSH_USER}@${host}" \
    "sudo -n -l poweroff >/dev/null || exit 1; nohup sh -c 'sleep 1; sudo -n poweroff' >/dev/null 2>&1 &" \
    >"${LOG_DIR}/${host}.log" 2>&1 &
  pids+=($!)
done

sent=()
failed=0
for i in "${!HOSTS[@]}"; do
  if wait "${pids[$i]}"; then
    sent+=("${HOSTS[$i]}")
  else
    echo "  FAILED: ${HOSTS[$i]}: $(cat "${LOG_DIR}/${HOSTS[$i]}.log")"
    failed=1
  fi
done

# A host counts as down once it stops answering ping.
for host in "${sent[@]}"; do
  deadline=$((SECONDS + DOWN_TIMEOUT))
  while ping -c 1 -W 1 "${host}" >/dev/null 2>&1; do
    if [ ${SECONDS} -ge ${deadline} ]; then
      echo "  STILL UP after ${DOWN_TIMEOUT}s: ${host}"
      failed=1
      continue 2
    fi
    sleep 1
  done
  echo "  down:   ${host}"
done

if [ ${failed} -eq 0 ]; then
  echo "All brains are down -- safe to switch off compute."
fi
exit ${failed}
