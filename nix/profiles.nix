{
  pkgs,
  config,
  lib,
  inputs,
  ...
}:
{
  profiles = {
    dev.module = import ./profiles/dev.nix {
      inherit pkgs config lib;
    };

    prod = {
      extends = [ "webui" ]; # give production access to web-ui profile
      module = import ./profiles/prod.nix {
        inherit pkgs config lib;
      };
    };

    webui.module = import ./profiles/webui.nix {
      inherit pkgs config lib;
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

    docs.module = import ./profiles/docs.nix {
      inherit pkgs inputs;
    };
  };
}
