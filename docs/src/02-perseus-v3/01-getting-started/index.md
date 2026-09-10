# Getting Started

This project is managed by [Nix](https://nixos.org/)! This makes getting started rather easy - all you need to do is install [Nix](https://nixos.org/) and [Devenv.sh](https://devenv.sh), which will take care of the rest. For your convenience, we have developed some scripts which handle all these installations for you. Whilst this page will guide you through the steps for getting everything installed and running on your machine, it is strongly recommended that you read through the rest of the docs (particularly the [Standards](../03-standards/index.md) section) to better understand the systems on Perseus.

# First Time Setup

The recommended way to set up our software environment is using the `member-setup.sh` script. This script has been tested on Debian-based distros (Ubuntu, for example) and should run on others as long as you install `git` with your system's package manager first[^1].

[^1]: If you are using NixOS then the script will detect this and show you the nix config to change.

## Using the Setup Script

First, make sure you have `curl` installed:

```
sudo apt update
sudo apt install curl
```

Then, you can download and call the script without cloning the repo:

```
curl https://raw.githubusercontent.com/ROAR-QUTRC/perseus-v3/refs/heads/main/software/scripts/member-setup.sh | bash
```

If the script succeeds, the perseus-v3 repo will be cloned and set up in your home directory at `~/perseus-v3`.

??? note "Cloning into another location"

    If you want the repo in another location, either follow the manual installation steps, or delete the repo and clone it again safely.

    The script also runs the command `devenv --profile dev allow` so that you are automatically dropped into a dev shell when you enter the directory. If you move the repo, you will need to run this command again from the new location.

## Manual Installation

This section assumes you are using a Debian-based distro. If you are not, you must replace the `apt` commands with your system's package manager.

1. Install `git` if it's not already present on your system by running `sudo apt install git`
2. Clone the `perseus-v3` repo with `git clone https://github.com/ROAR-QUTRC/perseus-v3.git`
3. Install Nix and devenv using the provided setup script: `./software/scripts/nix-setup.sh` (accept all options when prompted by typing `y` and pressing enter)
4. Enable devenv auto-activation inside the repo using the command `devenv --profile dev allow`[^2]
5. You should automatically enter the shell at this point, and several downloads - and potentially some builds - should begin

[^2]: You can replace the `dev` argument with the name of the devenv profile you want to use by default

### Setup script details

For the curious among you, the `nix-setup.sh` script, in order:

- [Installs Nix](https://github.com/DeterminateSystems/nix-installer)
- Configures some trusted substitutors for Nix to download as much as possible from binary caches
- Installs `devenv` using Nix
- Adds `devenv` to your `.zshrc` and `.bashrc` files
- Enables `devenv` auto-activation for the `dev` profile

!!! warning

    This script cannot set up devenv auto-activation if you're not using `bash` or `zsh` as your shell!
    You'll have to set up the hook yourself according to its [docs](https://devenv.sh/auto-activation/).

It's also designed to be able to run multiple times if something goes wrong - it checks whether the modifications it makes are already present, and if it detects them, skips that step.
