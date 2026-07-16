{
  pkgs,
  ...
}:
let
  buildConfig =
    {
      system,
      hostname ? "",
      username,
      ...
    }:
    let
      machineName = if hostname != "" then "${username}@${hostname}" else username;
      machineConfig = ./machines/${hostname}/default.nix;
      userConfig = ./users/${username}/default.nix;
    in
    {
      machines.${machineName} = {
        inherit system;
        home-manager = {
          home = {
            inherit username;
            homeDirectory = "/home/${username}";
            imports = [
              ./home
            ]
            ++ pkgs.lib.lists.optional ((hostname != "") && (builtins.pathExists machineConfig)) machineConfig
            ++ pkgs.lib.lists.optional (builtins.pathExists userConfig) userConfig;
          };
        };
      };
    };
  buildConfigs =
    # foldl' is just like array reduce in other languages - args are accumulator function,
    # initial value, and list to reduce
    configs: builtins.foldl' (acc: new: acc // new) { } (map buildConfig configs);
in
buildConfigs [
  # default to ARM64, and specify x86 systems manually
  {
    system = "aarch64-linux";
    username = "qutrc";
  }
  # and systems with manual config should also be specified
  {
    system = "aarch64-linux";
    username = "qutrc";
    hostname = "big-brain";
  }
  {
    system = "aarch64-linux";
    username = "qutrc";
    hostname = "medium-brain";
  }
  # personal systems
  {
    system = "aarch64-linux";
    username = "jcnic";
  }
  {
    system = "aarch64-linux";
    username = "dingo";
  }
]
