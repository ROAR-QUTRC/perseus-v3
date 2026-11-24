# devenv-setup.sh

## Motivation

This script was originally a part of nix-setup.sh in the
[perseus-v2](https://github.com/ROAR-QUTRC/perseus-v2) repo. It's been split to
its own file for readability and ease of use. It sets up devenv integration with
bash and zsh. If any members use other shells (e.g. nushell, fish), this script
can be expanded to include those.

## How it works

It checks for existence of ~/.bashrc and ~/.zshrc and appends the
[devenv hook](https://devenv.sh/auto-activation/) to them. It integrates with
Nix home-manager, prompting the user to add the hook to their configuration if
the config files are symlinks.
