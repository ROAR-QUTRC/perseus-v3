#!/usr/bin/env bash

shopt -s globstar # Enable globstar for recursive globbing

# cd to the repo root
cd "$(git rev-parse --show-toplevel)" || exit

rm -rf ./**/result
rm -rf ./**/build
rm -rf ./**/log
rm -rf ./**/install
rm -rf ./**/generated
rm -rf ./**/node_modules
