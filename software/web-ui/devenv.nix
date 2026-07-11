{pkgs, ...}:

{

  profiles.web-ui.module = {
  packages = with pkgs.gst_all_1; [
    gstreamer
    gst-plugins-base
    gst-plugins-good
    gst-plugins-bad
    gst-plugins-rs
  ];

  languages = {
    typescript.enable = true;
    javascript = {
      enable = true;
      corepack.enable = true;
    };
  };
  };
}