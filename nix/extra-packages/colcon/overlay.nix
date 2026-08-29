final: prev:
let
  # colcon with output filtered with grep to suppress the BARRAGE of warnings from colcon about the ament prefix path
  # Unfortunately, they're just a side effect of the nix ros build system
  colcon = final.callPackage ./colcon-no-warnings.nix { inherit (prev) colcon; };
  # mini package which puts COLCON_IGNORE in the output result folder
  # allows colcon build of workspace after run nix build
  colcon-ignore = final.callPackage ./colcon-ignore.nix { };
in
{
  inherit colcon colcon-ignore;
}
