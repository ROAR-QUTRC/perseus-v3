{ pkgs, config, ... }:
{
  profiles = {
    # Waiting for docs release but autoactivation should drop you into this shell
    default.module = import ./profiles/default.nix {
      inherit pkgs config;
    };

    web-ui.module = import ./profiles/web-ui.nix {
      inherit pkgs config;
    };

    simulation = {
      extends = [ "default" ];
      module = {

      };
    };
  };
}
