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

  packages = [
    docs-with-plugins
    pkgs-docs.drawio # still need the drawio binary for mkdocs-drawio-exporter
    pkgs-docs.xvfb # needed for a headless build of mkdocs-drawio-exporter
  ];
  languages = {
    python.enable = true;
  };

  scripts = {
    dev.exec = "cd $(git rev-parse --show-toplevel)/docs && mkdocs serve";
    build.exec = "cd $(git rev-parse --show-toplevel)/docs && xvfb-run -a mkdocs build";
  };
}
