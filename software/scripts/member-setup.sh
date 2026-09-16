#!/usr/bin/env bash

# If you're changing this file, make sure to read the perseus-v3/software/scripts docs

set -euo pipefail

# exit if run as root
if [ "$EUID" -eq 0 ]; then
  echo "Please run as yourself! Running as superuser (ie, with sudo) breaks the setup."
  exit 1
fi

# check for git
if ! command -v git >/dev/null 2>&1; then
  echo "Git is not installed."
  # Check if apt is available
  if command -v apt-get >/dev/null 2>&1; then
    echo "apt-get is available, using it to install git..."
    sudo apt-get update >/dev/null 2>&1
    sudo apt-get install -y git >/dev/null 2>&1
  else
    echo "apt-get is not available. Please install Git manually then rerun the script."
    exit 1
  fi
fi

# Clone the perseus-v3 repo
cd ~
if ! [ -d "perseus-v3" ]; then
  echo "Perseus repo not detected. Cloning now."
  git clone https://github.com/ROAR-QUTRC/perseus-v3.git
else
  echo "Perseus repo already cloned. Continuing."
fi

# Run the nix-setup script
echo "Running nix-setup.sh script. If asked, accept all config options by typing 'y', then press enter."
cd ~/perseus-v3
./software/scripts/nix-setup.sh

# Enable autoactivation for the base environment
devenv --profile dev allow

echo "Setup script ran successfully!"
echo "cd out and into the perseus-v3 directory to build the environment. This may perform several builds or downloads."
