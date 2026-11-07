{
  stdenv,
  cleanCmakeSource,
  cmake,
  hi-can-raw,
  ncurses,
}:

stdenv.mkDerivation rec {
  pname = "can_debugger";
  version = "0.0.1";

  src = cleanCmakeSource {
    src = ./.;
    name = pname;
  };

  nativeBuildInputs = [ cmake ];
  buildInputs = [
    hi-can-raw
    ncurses
  ];
}
