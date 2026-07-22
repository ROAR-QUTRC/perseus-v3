{ pkgs, config, ... }:
{
  profiles = {
    # Waiting for docs release but autoactivation should drop you into this shell
    base.module = import ./profiles/base.nix {
      inherit pkgs config;
    };

    web-ui.module = import ./profiles/web-ui.nix {
      inherit pkgs config;
    };

    cicd.module = import ./profiles/cicd.nix {
      inherit pkgs config;
    };

    firmware.module = import ./profiles/firmware.nix {
      inherit pkgs config;
    };

    simulation = {
      extends = [ "base" ];
      module = {

      };
    };

  };
}
