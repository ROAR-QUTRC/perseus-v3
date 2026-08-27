{ pkgs, ... }: {
  env.NO_MKDOCS_2_WARNING = 1;

  packages =
    with pkgs.python3Packages;
    [
      mkdocs-material
      mkdocs-material-extensions
      mkdocs-awesome-nav
      mkdocs-drawio-exporter
    ]
    ++ mkdocs-material.optional-dependencies.imaging;

  languages = {
    python.enable = true;
  };

  scripts = {
    dev.exec = "cd $(git rev-parse --show-toplevel)/docs && mkdocs serve";
    build.exec = "cd $(git rev-parse --show-toplevel)/docs && mkdocs build";
  };
}
