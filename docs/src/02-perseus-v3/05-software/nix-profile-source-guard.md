# Nix and its installed packages disappears from PATH inside tmux

## Symptom

`nix` and packages installed through `nix profile install` is not found on `PATH`. Re-sourcing the Nix profile script:

```bash
. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
```

Gives back the installed packages, but nix itself cannot be found on `PATH`.

## Background

The profile script `nix-daemon.sh` is sourced once per shell by `/etc/profile.d/nix.sh` and `/etc/bash.bashrc`. It:

- sets a guard variable, `__ETC_PROFILE_NIX_SOURCED=1`, and returns immediately if it is already set
- prepends two directories to `PATH`:
  - `~/.local/state/nix/profiles/profile/bin` - your per-user profile (programs installed with `nix profile install`)
  - `/nix/var/nix/profiles/default/bin` - the system default profile, which contains the `nix` binary itself

There are three profiles involved:

| Profile                      | Path                                  | Contains           |
| ---------------------------- | ------------------------------------- | ------------------ |
| System default               | `/nix/var/nix/profiles/default`       | `nix` itself       |
| Per-user                     | `~/.local/state/nix/profiles/profile` | your packages      |
| Legacy compatibility symlink | `~/.nix-profile`                      | points to per-user |

## Cause

tmux panes inherit the environment of the shell that started the tmux server, including `__ETC_PROFILE_NIX_SOURCED=1`. Because the guard travels through the environment, every shell inside tmux sees it and skips the script. Therefore the `PATH` inside tmux will not contain the path to `nix` and nix installed packages. 

## Diagnosis

Check whether the guard is set and which Nix directories are on `PATH`:

```bash
echo "SOURCED=${__ETC_PROFILE_NIX_SOURCED-unset}"
echo "$PATH" | tr ':' '\n' | grep nix
```

If `SOURCED=1` but `/nix/var/nix/profiles/default/bin` is missing from `PATH`, the guard was inherited from a parent shell and the script never ran.

## Fix

=== "Immediate"

    Unset the guard so the script runs again, then re-source it:

    ```bash
    unset __ETC_PROFILE_NIX_SOURCED
    . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    ```

=== "Permanent"

    Make every interactive shell self-healing by adding this to
    `~/.bashrc`. The `case` guards keep it idempotent no matter how many
    times `~/.bashrc` runs:

    ```bash
    # tmux panes inherit __ETC_PROFILE_NIX_SOURCED, which stops the
    # nix profile script from re-running; ensure its PATH entries exist.
    case ":$PATH:" in
      *":/nix/var/nix/profiles/default/bin:"*) ;;
      *) export PATH="/nix/var/nix/profiles/default/bin:$PATH" ;;
    esac
    case ":$PATH:" in
      *":$HOME/.local/state/nix/profiles/profile/bin:"*) ;;
      *) export PATH="$HOME/.local/state/nix/profiles/profile/bin:$PATH" ;;
    esac
    ```
