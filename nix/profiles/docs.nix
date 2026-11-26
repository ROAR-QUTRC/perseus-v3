{ pkgs, inputs, ... }:
let
  pkgs-docs = import inputs.nixpkgs-docs {
    system = pkgs.stdenv.system;
    config.allowUnfree = true; # allow_unfree in devenv.yaml only covers the default nixpkgs
  };
  # docs-with-plugins is a python3 environment with mkdocs and its plugins so they can see each other
  docs-with-plugins = pkgs-docs.python3.withPackages (
    ps:
    with ps;
    [
      mkdocs
      mkdocs-material
      mkdocs-material-extensions
      mkdocs-awesome-nav
      mkdocs-drawio-exporter
    ]
    ++ mkdocs-material.optional-dependencies.imaging
  );
in
{
  env.NO_MKDOCS_2_WARNING = 1;
  # (stops the 'Failed to load module "canberra-gtk-module"' message when building locally after fixed the mesa loader)
  env.GTK_PATH = "${pkgs-docs.libcanberra-gtk3}/lib/gtk-3.0";

  packages = [
    docs-with-plugins
    pkgs-docs.drawio # still need the drawio binary for mkdocs-drawio-exporter
    pkgs-docs.xvfb-run # drawio is Electron and needs an X display or else it runs into error on ci
    pkgs-docs.dbus # provides temp dbus-run-session, a throwaway session bus so chromium stops showing dbus errors
    pkgs-docs.libcanberra-gtk3 # GTK sound plugin, see env.GTK_PATH above
  ];
  languages = {
    python.enable = true;
  };

  scripts = {
    dev.exec = "cd $(git rev-parse --show-toplevel)/docs && mkdocs serve";
    # the nix dbus package looks for its session config in /etc, so point it at the copy in the store else it wont use the dbus
    build.exec = "cd $(git rev-parse --show-toplevel)/docs && xvfb-run -a dbus-run-session --config-file=${pkgs-docs.dbus}/share/dbus-1/session.conf mkdocs build";
  };
}
