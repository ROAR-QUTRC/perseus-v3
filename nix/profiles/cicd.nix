# This profile is for CI/CD testing ONLY

{ ... }: {
  extends = [
    "prod" # this extends webui already
    "firmware"
  ];

  module = {
    # Add CI/CD tests here
  };
}
