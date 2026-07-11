#!/bin/sh

# Add devenv hooks to bash and zsh if not already present
if grep -Fq 'devenv hook bash' ~/.bashrc >/dev/null 2>&1; then
  echo "bash devenv hook already set up!"
elif command -v bash >/dev/null 2>&1; then
  if [ -L ~/.bashrc ]; then
    echo "\~/.bashrc is a symlink. If you are using home-manager, add this to your home.nix:"
    # shellcheck disable=SC2016
    echo 'programs.bash.initExtra = \"eval $(devenv hook bash)\";'
  else
    echo "Setting up bash devenv hook..."
    # shellcheck disable=SC2016
    echo 'eval "$(devenv hook bash)"' >>~/.bashrc
  fi
else
  echo "bash not detected on system, not setting up .bashrc"
fi

if grep -Fq 'devenv hook zsh' ~/.zshrc >/dev/null 2>&1; then
  echo "zsh devenv hook already set up!"
elif command -v zsh >/dev/null 2>&1; then
  if [ -L ~/.zshrc ]; then
    echo "\~/.zshrc is a symlink. If you are using home-manager, add this to your home.nix:"
    # shellcheck disable=SC2016
    echo 'programs.zsh.initExtra = \"eval $(devenv hook zsh)\";'
  else
    echo "Setting up zsh devenv hook..."
    # shellcheck disable=SC2016
    echo 'eval "$(devenv hook zsh)"' >>~/.zshrc
  fi
else
  echo "zsh not detected on system, not setting up .zshrc"
fi
