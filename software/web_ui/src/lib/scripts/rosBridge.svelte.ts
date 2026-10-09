import * as ROSLIB from "roslib";
import { env as publicEnv } from "$env/dynamic/public";

// Must match ROSBRIDGE_PORT in devenv.nix (default 9090). Read from the UI server's
// runtime environment (injected into the page per request), so the server must run
// inside the devenv shell (yarn host / yarn start) to pick it up.
const rosbridgePort = publicEnv.PUBLIC_ROSBRIDGE_PORT || "9090";

let rosConnectionWrap = $state<ROSLIB.Ros | false>(false);
let retryCounter: number = 0;
let retryTimeoutHandle: NodeJS.Timeout | null = null;

export const getRosConnection = () => rosConnectionWrap;

export const connectRos = (address: string) => {
  const rosConnection = new ROSLIB.Ros({
    url: `ws://${address}:${rosbridgePort}`,
  });

  // print on connection, close and error
  rosConnection.on("connection", () => {
    console.log("rosbridge connected");
    if (retryTimeoutHandle) {
      clearTimeout(retryTimeoutHandle);
      retryTimeoutHandle = null;
      retryCounter = 0;
    }
    rosConnectionWrap = rosConnection;
  });
  rosConnection.on("error", (error) => {
    console.log("Error connecting to websocket server: ", error);
  });
  rosConnection.on("close", () => {
    console.log("rosbridge disconnected");
    // Make sure no timeout is already running
    rosConnectionWrap = false;
    if (retryTimeoutHandle) {
      clearTimeout(retryTimeoutHandle);
      retryTimeoutHandle = null;
    }

    if (retryCounter === -1) {
      // If the connection was closed manually, don't retry
      retryCounter = 0;
      return;
    } else {
      // Retry every 2^n seconds, up to 32 seconds
      let timeoutTime = Math.pow(2, retryCounter) * 1000;
      retryTimeoutHandle = setTimeout(() => {
        console.log(`Retrying connection in ${timeoutTime} seconds...`);
        connectRos(address);
      }, timeoutTime);
      if (timeoutTime <= 16000) retryCounter++; // this limits the timeout to 32 seconds
    }
  });
};

export const disconnectRos = () => {
  if (rosConnectionWrap) {
    rosConnectionWrap.close();
  }
  retryCounter = -1; // This is used a signal that disconnection was manual
};
