{ pkgs, ... }:
{
  enterShell = ''
    # onnxruntime ships libonnxruntime.pc in its dev output, which the
    # workspace buildEnv does not surface. Add it so vision's
    # pkg_check_modules(libonnxruntime) resolves. The .pc uses absolute
    # paths, so this also supplies the correct lib flags.
    export PKG_CONFIG_PATH="${pkgs.onnxruntime.dev}/lib/pkgconfig''${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    # vision includes <onnxruntime/onnxruntime_cxx_api.h>, but the .pc
    # only advertises the .../include/onnxruntime subdir. Nix build inputs
    # normally get -isystem $dev/include automatically; replicate that here
    # so the "onnxruntime/"-prefixed include resolves.
    export CPATH="${pkgs.onnxruntime.dev}/include''${CPATH:+:$CPATH}"
  '';
}
