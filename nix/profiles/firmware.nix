{ pkgs, ... }:
{
  packages = with pkgs; [
    platformio
    avrdude
    esptool
    espflash
  ];

  enterShell = ''
    cd $DEVENV_ROOT/firmware/excavation-bucket
  '';
}
