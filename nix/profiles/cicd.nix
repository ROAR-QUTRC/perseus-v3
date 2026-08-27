# This profile is for CI/CD testing ONLY

{ ... }: {
  extends = [
    "prod"
    "web-ui"
    "firmware"
  ];

  module = {
    # Add CI/CD tests here
  };
}
