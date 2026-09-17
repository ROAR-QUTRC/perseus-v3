#!/usr/bin/env bash

shopt -s globstar # Enable globstar for recursive globbing

# cd to the repo root
cd "$(git rev-parse --show-toplevel)" || exit

echo "[1/6] Removing result directories..."
rm -rf ./**/result
echo "[2/6] Removing build directories..."
rm -rf ./**/build
echo "[3/6] Removing log directories..."
rm -rf ./**/log
echo "[4/6] Removing install directories..."
rm -rf ./**/install
echo "[5/6] Removing generated directories..."
rm -rf ./**/generated
echo "[6/6] Removing node_modules directories..."
rm -rf ./**/node_modules
echo "Cleanup complete!"
