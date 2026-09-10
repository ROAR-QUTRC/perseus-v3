# Devenv Configuration

This project uses devenv.sh as a convenient and more intuitive wrapper around Nix which still provides us with the same level of control and reproducibility over the software environment. You can find an [API reference](https://devenv.sh/reference/environment-variables/) and various [guides](https://devenv.sh/getting-started/) on their [docs website](https://devenv.sh).

## Config Files

The configuration can be found in several places:

- `devenv.nix`, `devenv.yaml` in the project root
- The contents of the `nix/` folder in the repository
- Various `default.nix` package definition and `overlay.nix` files placed near relevant code

### Main Config Files

The `devenv.yaml` configures devenv itself and defines the nixpkgs inputs (similar to `flake.nix` inputs) used by the environment. Moreover, in this project, `devenv.nix` is intended to only contain configuration that applies to the entire system (for example, importing overlays). Instead of putting configuration into the main `devenv.nix` environment, everything is abstracted into a [profile](https://devenv.sh/profiles/), allowing easy isolation of tech stacks.

### The `nix/` Folder

This folder is where most of the Nix-related code is organised, based on the file tree here:

```bash
nix
├── extra-packages # (1)!
│   ├── colcon
│   │   └── colcon plugins and patches
│   ├── patches
│   │   └── manually patched packages
│   └── ros-packages
│       └── auto generated with nix2ros
├── machines # (2)!
│   └── home
│       └── home-manager Nix config
├── profiles # (3)!
│   └── profile-name.nix
├── overlays.nix # (4)!
├── profiles.nix # (5)!
├── ros-workspace.nix # (6)!
└── treefmt.nix # (7)!
```

1. All the Nix packages that we maintain or patch live in here. At the time of writing, this includes the ROS2 packages that we develop for Perseus, along with some patches/plugins for colcon and third-party ROS2 packages.
2. Machines refers to the compute units that we configure using [home-manager](https://github.com/nix-community/home-manager).
3. Each profile is defined in its own file, which can be found in this folder.
4. This is the main `overlay.nix`, which is imported by devenv, connecting it to the overlay tree in the repository.
5. Similar to the `overlay.nix` file above, `profiles.nix` imports all the profile files.
6. Some shared logic for setting up ROS2 environments has been abstracted here from the dev and prod profiles.
7. Configuration for the `treefmt` formatter.

### Other Files

The remaining config files take two forms:

- Nix package derivations for native and shared software
- [Nix overlays](https://nixos.wiki/wiki/Overlays)

These files are scattered throughout the codebase because it makes more organisational sense to place them next to the code they relate to. Nix overlays can be somewhat confusing at first, but in essence, they add our packages to nixpkgs and modify existing packages in nixpkgs.

## Devenv Maintenance

Most of the maintenance required to keep the devenv environment running smoothly is periodically running `devenv gc` to delete unused builds and save disk space. Running `devenv update` to update the lock file should only be done by someone with a good understanding of the whole repository, as it may potentially break dependencies.

## Profiles
