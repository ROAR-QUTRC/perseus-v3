{
  pkgs,
  config,
  lib,
  ...
}:
{
  profiles = {
    # Keyed by hostname and activated automatically on the machine it names, so these
    # apply whatever --profile was asked for. See the file for what that is used for.
    hostname = import ./profiles/hostnames.nix;

    dev.module = import ./profiles/dev.nix {
      inherit pkgs config lib;
    };

    prod.module = import ./profiles/prod.nix {
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
      extends = [ "dev" ];
      module = import ./profiles/autonomy.nix {
        inherit pkgs config;
      };
    };

    simulation = {
      extends = [ "dev" ];
      module = {

      };
    };

    cyclonedds = {
      extends = [ "dev" ];
      module = import ./profiles/cyclonedds.nix {
        inherit pkgs config;
      };
    };

  };
}
