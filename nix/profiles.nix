{
  pkgs,
  config,
  lib,
  ...
}:
{
  profiles = {
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

  };
}
