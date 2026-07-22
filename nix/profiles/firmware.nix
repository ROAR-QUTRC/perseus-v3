{ pkgs, ... }: {
  packages = with pkgs; [
    platformio
    avrdude
    esptool
    espflash
  ];

  # languages.python.enable = true;

  enterShell = ''
    cd $DEVENV_ROOT/firmware/excavation-bucket
  '';
}
