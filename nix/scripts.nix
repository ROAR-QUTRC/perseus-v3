{ pkgs, ... }:
{
  tasks = {
    "scripts:update".exec = "${pkgs.scripts.update}/bin/update";
  };
  scripts = {
    build.exec = "echo 'Hello, World!'";
  };
}
