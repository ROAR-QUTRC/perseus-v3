{ pkgs, python3Packages }:
python3Packages.buildPythonApplication {
  pname = "camera_server";
  version = "0.0.1";

  src = ./.;
  format = "setuptools";

  nativeBuildInputs = with pkgs; [
    gobject-introspection
  ];

  propagatedBuildInputs = with python3Packages; [
    python-socketio
    gst-python
    pygobject3
    requests
    websocket-client
  ];
}
