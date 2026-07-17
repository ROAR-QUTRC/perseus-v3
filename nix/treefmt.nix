# treefmt.nix
{ ... }:
{
  treefmt = {
    enable = true;

    config = {

      # Used to find the project root
      projectRootFile = "devenv.nix";

      # enable formatters and linters
      programs.actionlint.enable = true;
      programs.clang-format.enable = true;
      programs.cmake-format.enable = true;
      programs.dos2unix.enable = true;
      programs.keep-sorted.enable = true;
      programs.ruff-check.enable = true;
      programs.ruff-format.enable = true;
      programs.shellcheck.enable = true;
      programs.shfmt.enable = true;
      # TODO: Look into enabling mypy
      programs.nixfmt.enable = true;
      programs.prettier = {
        enable = true;
        settings = {
          pluginSearchDirs = [
            "./software/web_ui"
          ];
        };
      };
      programs.taplo.enable = true;
      programs.typos.enable = true;
      programs.yamlfmt.enable = true;

      # config
      settings = {
        excludes = [
          # build dirs
          ".devenv/**"
          "build/**"
          "result/**"
          # generated stuff
          "generated/**"
          "treefmt.toml"
          # docs
          "docs/figures-source/**"
          "docs/pyproject/**"
          "docs/source/_static/fonts/**"
          "docs/source/robots.txt"
          "docs/source/intersphinx/**"
          # config
          "*.rviz"
          "setup.cfg"
          # ROS stuff
          "software/ros_ws/src/**/resource/**"
          # non-formattable file types
          "*.drawio"
          "*.svg"
          "*.ttf"
          "*.png"
          "*.jpg"
          "*.jpeg"
          "*.gif"
          "*.bmp"
          "*.ico"
          "*.pdf"
          "*sdkconfig*"
          # meshes
          "software/ros_ws/src/**/meshes/**"
          "*.dae"
          "*.stl"
          "*.step"
          "*.3mf"
          # ML models
          "*.onnx"
          # PCBs (KiCAD)
          "hardware/**"
          # miscellaneous
          "*.lock"
          "*.patch"
          "package-lock.json"
          "go.mod"
          "go.sum"
          ".gitattributes"
          ".gitignore"
          ".gitmodules"
          ".hgignore"
          ".svnignore"
          "LICENSE"
        ];
        formatter = {
          cmake-format = {
            includes = [
              "CMakeLists.txt"
              "**/CMakeLists.txt"
              "**/*.cmake"
            ];
            options = [ "--in-place" ];
          };
          typos = {
            excludes = [
              "typos.toml"
              "**/treefmt.nix"
            ];
            includes = [ "*" ];
          };
          actionlint = {
            includes = [
              ".github/workflows/*.yml"
              ".github/workflows/*.yaml"
            ];
          };
          clang-format = {
            includes = [
              "*.c"
              "*.cc"
              "*.cpp"
              "*.h"
              "*.hh"
              "*.hpp"
              "*.glsl"
              "*.vert"
              ".tesc"
              ".tese"
              ".geom"
              ".frag"
              ".comp"
            ];
            options = [ "-i" ];
          };
          dos2unix = {
            includes = [ "*" ];
            options = [ "--keepdate" ];
          };
          keep-sorted = {
            includes = [ "*" ];
          };
          nixfmt = {
            includes = [ "*.nix" ];
          };
          prettier = {
            includes = [
              "*.cjs"
              "*.css"
              "*.html"
              "*.js"
              "*.json"
              "*.json5"
              "*.jsx"
              "*.md"
              "*.mdx"
              "*.mjs"
              "*.scss"
              "*.ts"
              "*.tsx"
              "*.vue"
              "*.yaml"
              "*.yml"
            ];
            options = [
              "--write"
              "--config"
              "/nix/store/08dd0r4gskvj94nzfkppinw9z0393jnd-prettierrc.json"
            ];
          };
          ruff-format = {
            includes = [
              "*.py"
              "*.pyi"
            ];
          };
          shellcheck = {
            includes = [
              "*.sh"
              "*.bash"
              "*.envrc"
              "*.envrc.*"
            ];
          };
          shfmt = {
            includes = [
              "*.sh"
              "*.bash"
              "*.envrc"
              "*.envrc.*"
            ];
            options = [
              "-w"
              "-i"
              "2"
              "-ci"
            ];
          };
          taplo = {
            includes = [ "*.toml" ];
            options = [ "format" ];
          };
          yamlfmt = {
            includes = [
              "*.yaml"
              "*.yml"
            ];
          };
        };
      };
    };
  };
}
