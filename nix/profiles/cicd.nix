# This profile is for CI/CD testing ONLY

{ ... }: {
  extends = [
    "dev"
    "prod"
    "web-ui"
    "simulation"
    "autonomy"
    "firmware"
  ];

  module = {
    # Add CI/CD tests here
  };
}
