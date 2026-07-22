final: prev:
let
  build-wrapped-script =
    {
      name,
      deps ? [ ],
    }:
    prev.runCommandLocal name { nativeBuildInputs = [ prev.mkWrapper ]; } ''
      makeWrapper ${./${name}} $out/bin/${name} \
        --prefix PATH : ${prev.lib.makeBinPath deps}
    '';
in
{
  scripts = {
    clean = build-wrapped-script {
      name = "clean.bash";
      deps = with prev; [ git ];
    };
    nix-package = build-wrapped-script {
      name = "nix-package.sh";
      deps = with prev; [ git ];
    };
    vcan-setup = build-wrapped-script {
      name = "nix-package.sh";
      deps = with prev; [
        kmod
        iproute2
      ];
    };
    # nix-setup and devenv-setup aren't needed here because they are run before nix is installed
  };
}
