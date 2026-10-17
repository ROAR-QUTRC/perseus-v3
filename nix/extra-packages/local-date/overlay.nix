# Exposes the base station SNTP time server (github:dbreth/rs-local-time-server) as pkgs.local-date
local-date: final: prev: {
  local-date = local-date.packages.${final.stdenv.hostPlatform.system}.default;
}
