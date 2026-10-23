# Groot2: BehaviorTree.CPP editor and live monitor (https://www.behaviortree.dev/groot)
# Proprietary freeware, only distributed as an AppImage, so it is wrapped rather than built.
{
  lib,
  stdenv,
  appimageTools,
  fetchurl,
}:
let
  pname = "groot2";
  version = "1.9.0";
  sources = {
    x86_64-linux = {
      arch = "x86_64";
      hash = "sha256-JtBAJVyxRixlEZVPNF7nMoDXLH+IovnRO7r/mQLebHA=";
    };
    aarch64-linux = {
      arch = "aarch64";
      hash = "sha256-YM7q/sWRVXMmpK/sKIFrQK51th21cSaxOSrnK3i6ZhU=";
    };
  };
  source =
    sources.${stdenv.hostPlatform.system}
      or (throw "groot2: unsupported system ${stdenv.hostPlatform.system}");
  src = fetchurl {
    url = "https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v${version}-${source.arch}.AppImage";
    inherit (source) hash;
  };
  appimageContents = appimageTools.extractType2 { inherit pname version src; };
in
appimageTools.wrapType2 {
  inherit pname version src;

  # The sandbox passes the host's /etc/fonts through but replaces /usr, so on a non-NixOS
  # host /usr/share/fonts (which that config names) is empty and every glyph renders as a
  # box. Shipping a font in the sandbox's own /usr/share/fonts gives the host config one.
  extraPkgs = pkgs: [ pkgs.dejavu_fonts ];

  extraInstallCommands = ''
    for desktop in ${appimageContents}/*.desktop; do
      install -Dm444 "$desktop" $out/share/applications/${pname}.desktop
    done
    substituteInPlace $out/share/applications/${pname}.desktop \
      --replace-quiet 'Exec=AppRun' 'Exec=${pname}'
    if [ -d ${appimageContents}/usr/share/icons ]; then
      cp -r ${appimageContents}/usr/share/icons $out/share/
    fi
  '';

  meta = {
    description = "Editor and live monitor for BehaviorTree.CPP behavior trees";
    homepage = "https://www.behaviortree.dev/groot";
    license = lib.licenses.unfree;
    sourceProvenance = [ lib.sourceTypes.binaryNativeCode ];
    platforms = builtins.attrNames sources;
    mainProgram = pname;
  };
}
