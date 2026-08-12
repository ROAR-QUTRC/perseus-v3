#!/usr/bin/env bash

# If you're changing this file, make sure to read the perseus-v3/software/scripts docs

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit 1
fi

# Update and install required packages
sudo apt-get update >/dev/null 2>&1
sudo apt-get install -y git gh >/dev/null 2>&1

# Sign into gh CLI
if ! gh auth status >/dev/null 2>&1; then
  echo "You need to sign into github CLI. Follow these instructions:"
  gh auth login -w
else
  echo "GitHub CLI already logged in."
fi

# Clone the perseus-v3 repo
cd ~
if ! [ -d "perseus-v3" ]; then
  echo "Perseus repo not detected. Cloning now."
  gh repo clone ROAR-QUTRC/perseus-v3
else
  echo "Perseus repo already cloned. Continuing."
fi

# Run the nix-setup script
echo "Running nix-setup.sh script. If asked, accept all config options by typing 'y', then press enter."
cd ~/perseus-v3
./software/scripts/nix-setup.sh

# Enable autoactivation for the base environment
devenv allow --profile base

echo "Setup script ran successfully!"
echo "cd out and into the perseus-v3 directory to build the environment. This may perform several builds or downloads."
