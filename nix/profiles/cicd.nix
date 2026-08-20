# This profile is for CI/CD testing ONLY

{ ... }: {
  extends = [
    "base"
    "web-ui"
    "simulation"
    "autonomy"
    "firmware"
  ];

  module = {
    # Add CI/CD tests here
  };
}
