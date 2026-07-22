{
  pkgs,
  config,
  lib,
  ...
}:
{
  profiles = {
    # Waiting for docs release but autoactivation should drop you into this shell
    base.module = import ./profiles/base.nix {
      inherit pkgs config lib;
    };

    web-ui.module = import ./profiles/web-ui.nix {
      inherit pkgs config;
    };

    cicd = import ./profiles/cicd.nix {
      inherit pkgs config;
    };

    firmware.module = import ./profiles/firmware.nix {
      inherit pkgs config;
    };

    autonomy = {
      extends = [ "base" ];
      module = import ./profiles/autonomy.nix {
        inherit pkgs config;
      };
    };

    simulation = {
      extends = [ "base" ];
      module = {

      };
    };

  };
}
